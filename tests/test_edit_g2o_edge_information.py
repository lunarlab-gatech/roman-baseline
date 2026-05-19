import unittest

import gtsam
import numpy as np

from roman.offline_rpgo.edit_g2o_edge_information import (
    edit_g2o_edge_information,
    std_dev_to_information_matrix,
    std_dev_to_information_matrix_str,
)

_A = [gtsam.symbol('a', i) for i in range(5)]

# Original information matrix: identity (t_std=r_std=1.0)
_ORIG_INFO = "1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1"

# New std devs used when calling edit_g2o_edge_information in tests
_NEW_T_STD = 0.5
_NEW_R_STD = 0.5


def _edge(v0, v1):
    return f"EDGE_SE3:QUAT {v0} {v1} 1 0 0 0 0 0 1 {_ORIG_INFO}\n"


def _info_tokens(line: str) -> list:
    """Return the 21 information-matrix tokens (positions 10-30) of an edge line."""
    return line.strip().split()[10:]


def _expected_info_tokens() -> list:
    """Tokens for the new information matrix after editing."""
    I = std_dev_to_information_matrix(_NEW_T_STD, _NEW_R_STD)
    s = std_dev_to_information_matrix_str(information_matrix=I)
    return s.split()


class TestEditG2oEdgeInformation(unittest.TestCase):
    """
    Tests for edit_g2o_edge_information covering:
      - loop_closures=True with # LC: marker (updates even adjacent-index LC edges)
      - loop_closures=True without marker, non-adjacent (fallback: updates)
      - loop_closures=True without marker, adjacent (fallback: does NOT update — treated as odom)
      - odometry=True updates unmarked adjacent edges
      - odometry=True does NOT update # LC:-marked edges
      - Non-edge lines pass through unchanged
      - Line count is always preserved
    """

    def setUp(self):
        # Odometry edge: adjacent, no marker
        self.odom_edge = _edge(_A[0], _A[1])

        # LC edge marked with # LC:, but adjacent indices (the previously-dropped case)
        self.marked_lc_marker = "# LC:\n"
        self.marked_lc_edge   = _edge(_A[1], _A[2])   # |idx|=1, but preceded by # LC:

        # LC edge without marker, non-adjacent (old-format fallback)
        self.unmarked_lc_edge = _edge(_A[0], _A[3])   # |idx|=3, no marker

        # Non-edge lines that must always pass through unchanged
        self.vertex_line  = f"VERTEX_SE3:QUAT {_A[0]} 0 0 0 0 0 0 1\n"
        self.comment_line = "# NEW LOOP CLOSURES\n"

        self.all_lines = [
            self.comment_line,
            self.vertex_line,
            self.odom_edge,
            self.marked_lc_marker,
            self.marked_lc_edge,
            self.unmarked_lc_edge,
        ]

    # ------------------------------------------------------------------
    # loop_closures=True
    # ------------------------------------------------------------------

    def test_lc_mode_updates_marked_adjacent_lc_edge(self):
        """# LC:-marked edge with adjacent indices must be updated (not treated as odometry)."""
        result = edit_g2o_edge_information(
            self.all_lines, _NEW_T_STD, _NEW_R_STD, loop_closures=True)
        marked_lc_idx = result.index(self.marked_lc_marker.strip())
        updated_line = result[marked_lc_idx + 1]
        self.assertEqual(_info_tokens(updated_line), _expected_info_tokens())

    def test_lc_mode_updates_unmarked_nonadjacent_lc_edge(self):
        """Unmarked non-adjacent edge must be updated (fallback path)."""
        result = edit_g2o_edge_information(
            self.all_lines, _NEW_T_STD, _NEW_R_STD, loop_closures=True)
        prefix = f"EDGE_SE3:QUAT {_A[0]} {_A[3]}"
        updated = next(l for l in result if l.startswith(prefix))
        self.assertEqual(_info_tokens(updated), _expected_info_tokens())

    def test_lc_mode_does_not_update_unmarked_adjacent_odom_edge(self):
        """Unmarked adjacent edge must NOT be updated when loop_closures=True."""
        result = edit_g2o_edge_information(
            self.all_lines, _NEW_T_STD, _NEW_R_STD, loop_closures=True)
        prefix = f"EDGE_SE3:QUAT {_A[0]} {_A[1]}"
        unchanged = next(l for l in result if l.startswith(prefix))
        self.assertEqual(_info_tokens(unchanged), _ORIG_INFO.split())

    # ------------------------------------------------------------------
    # odometry=True
    # ------------------------------------------------------------------

    def test_odom_mode_updates_adjacent_unmarked_edge(self):
        """Unmarked adjacent edge must be updated when odometry=True."""
        result = edit_g2o_edge_information(
            self.all_lines, _NEW_T_STD, _NEW_R_STD, odometry=True)
        prefix = f"EDGE_SE3:QUAT {_A[0]} {_A[1]}"
        updated = next(l for l in result if l.startswith(prefix))
        self.assertEqual(_info_tokens(updated), _expected_info_tokens())

    def test_odom_mode_does_not_update_marked_lc_edge(self):
        """# LC:-marked edge must NOT be updated when odometry=True."""
        result = edit_g2o_edge_information(
            self.all_lines, _NEW_T_STD, _NEW_R_STD, odometry=True)
        marked_lc_idx = result.index(self.marked_lc_marker.strip())
        unchanged = result[marked_lc_idx + 1]
        self.assertEqual(_info_tokens(unchanged), _ORIG_INFO.split())

    def test_odom_mode_does_not_update_unmarked_nonadjacent_edge(self):
        """Unmarked non-adjacent edge must NOT be updated when odometry=True."""
        result = edit_g2o_edge_information(
            self.all_lines, _NEW_T_STD, _NEW_R_STD, odometry=True)
        prefix = f"EDGE_SE3:QUAT {_A[0]} {_A[3]}"
        unchanged = next(l for l in result if l.startswith(prefix))
        self.assertEqual(_info_tokens(unchanged), _ORIG_INFO.split())

    # ------------------------------------------------------------------
    # General invariants
    # ------------------------------------------------------------------

    def test_line_count_preserved_lc_mode(self):
        result = edit_g2o_edge_information(
            self.all_lines, _NEW_T_STD, _NEW_R_STD, loop_closures=True)
        self.assertEqual(len(result), len(self.all_lines))

    def test_line_count_preserved_odom_mode(self):
        result = edit_g2o_edge_information(
            self.all_lines, _NEW_T_STD, _NEW_R_STD, odometry=True)
        self.assertEqual(len(result), len(self.all_lines))

    def test_non_edge_lines_unchanged(self):
        """Vertex and comment lines must be identical in all modes."""
        for mode_kwargs in [{"loop_closures": True}, {"odometry": True}]:
            result = edit_g2o_edge_information(
                self.all_lines, _NEW_T_STD, _NEW_R_STD, **mode_kwargs)
            self.assertIn(self.vertex_line.strip(),  result)
            self.assertIn(self.comment_line.strip(), result)


if __name__ == "__main__":
    unittest.main()
