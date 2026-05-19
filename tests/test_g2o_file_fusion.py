import os
import tempfile
import unittest

import gtsam

from roman.offline_rpgo.g2o_file_fusion import create_config, g2o_file_fusion

# align.g2o uses plain integer vertex indices (not gtsam symbols); they are
# remapped to gtsam symbols by reformat_g2o_edge_lines.
_INFO = "1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1"

# gtsam symbols for robot 'a' after fusion
_A = [gtsam.symbol('a', i) for i in range(4)]


def _vertex(idx, x=0.0):
    return f"VERTEX_SE3:QUAT {idx} {x} 0 0 0 0 0 1"


def _edge(v0, v1, tx=1.0):
    return f"EDGE_SE3:QUAT {v0} {v1} {tx} 0 0 0 0 0 1 {_INFO}"


class TestG2oFileFusion(unittest.TestCase):
    """
    Tests for g2o_file_fusion covering:
      - add_lc_comment_markers=False: output identical to pre-change behavior
      - add_lc_comment_markers=True: every LC edge preceded by '# LC:'
      - Removing '# LC:' lines from marked output equals plain output
      - Odometry edges are never preceded by '# LC:'
      - LC edges that fail the threshold are dropped in both modes
    """

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        robot = "robot_a"

        # Odometry g2o: plain integer indices, 4 vertices, 3 odometry edges
        odom_lines = "\n".join(
            [_vertex(i, x=float(i)) for i in range(4)]
            + [_edge(i, i + 1) for i in range(3)]
        ) + "\n"

        # Single-robot LC g2o (align.g2o format): '# LC: N' before each edge.
        # LC1: idx 0->2 (non-adjacent), 5 associations — above any threshold we test.
        # LC2: idx 1->3 (non-adjacent), 2 associations — below thresh=3.
        # Note: self_lc=True skips edges where line[1] >= line[2], so we keep v0 < v1.
        lc_lines = "\n".join([
            "# LC: 5",
            _edge(0, 2, tx=2.0),
            "# LC: 2",
            _edge(1, 3, tx=3.0),
        ]) + "\n"

        def write(name, content):
            path = os.path.join(self.tmp, name)
            with open(path, "w") as f:
                f.write(content)
            return path

        odom_file = write(f"{robot}.g2o", odom_lines)
        lc_file   = write("align.g2o",   lc_lines)

        align_dir = self.tmp
        self.config = {
            'robots': [{'robot': robot, 'letter': 'a'}],
            'odometry': [{'robot': robot, 'file': odom_file}],
            'single_lc': [{'robot': robot, 'file': lc_file}],
            'multi_lc': [],
        }
        self.out_plain  = os.path.join(self.tmp, "out_plain.g2o")
        self.out_marked = os.path.join(self.tmp, "out_marked.g2o")

    def _read(self, path):
        with open(path) as f:
            return [l.rstrip('\n') for l in f.readlines()]

    def _call(self, add_lc_comment_markers, thresh=None, out=None):
        if out is None:
            out = self.out_marked if add_lc_comment_markers else self.out_plain
        g2o_file_fusion(self.config, out, thresh=thresh,
                        add_lc_comment_markers=add_lc_comment_markers)
        return self._read(out)

    def _is_edge(self, line, v0, v1):
        """Return True if line is an EDGE_SE3:QUAT between v0 and v1 (whitespace-agnostic)."""
        tokens = line.split()
        return (len(tokens) >= 3 and tokens[0] == "EDGE_SE3:QUAT"
                and int(tokens[1]) == v0 and int(tokens[2]) == v1)

    # ------------------------------------------------------------------
    # Backward compatibility
    # ------------------------------------------------------------------

    def test_plain_and_marked_identical_except_lc_comment_lines(self):
        """Removing all '# LC:' lines from marked output must equal plain output."""
        plain  = self._call(add_lc_comment_markers=False)
        marked = self._call(add_lc_comment_markers=True)
        stripped = [l for l in marked if l.strip() != "# LC:"]
        self.assertEqual(plain, stripped)

    def test_plain_has_no_lc_comment_lines(self):
        """Plain output must contain no '# LC:' lines."""
        plain = self._call(add_lc_comment_markers=False)
        self.assertEqual([l for l in plain if l.strip() == "# LC:"], [])

    # ------------------------------------------------------------------
    # Marked mode
    # ------------------------------------------------------------------

    def test_lc_edges_preceded_by_lc_comment(self):
        """Every kept LC edge must be preceded by '# LC:' in marked output."""
        marked = self._call(add_lc_comment_markers=True)
        lc_pairs = [(_A[0], _A[2]), (_A[1], _A[3])]
        for v0, v1 in lc_pairs:
            for i, line in enumerate(marked):
                if self._is_edge(line, v0, v1):
                    prev = marked[i - 1] if i > 0 else ""
                    self.assertEqual(prev.strip(), "# LC:",
                        msg=f"LC edge {v0}->{v1} not preceded by '# LC:'")
                    break
            else:
                self.fail(f"Expected LC edge {v0}->{v1} not found in output")

    def test_odometry_edges_not_preceded_by_lc_comment(self):
        """Odometry edges must never be preceded by '# LC:'."""
        marked = self._call(add_lc_comment_markers=True)
        odom_pairs = [(_A[0], _A[1]), (_A[1], _A[2]), (_A[2], _A[3])]
        for v0, v1 in odom_pairs:
            for i, line in enumerate(marked):
                if self._is_edge(line, v0, v1):
                    prev = marked[i - 1] if i > 0 else ""
                    self.assertNotEqual(prev.strip(), "# LC:",
                        msg=f"Odometry edge {v0}->{v1} incorrectly preceded by '# LC:'")
                    break

    # ------------------------------------------------------------------
    # Threshold filtering
    # ------------------------------------------------------------------

    def test_threshold_drops_low_association_lc_in_plain_mode(self):
        """LC2 (2 associations) must be absent when thresh=3 in plain mode."""
        plain = self._call(add_lc_comment_markers=False, thresh=3,
                           out=os.path.join(self.tmp, "out_thresh_plain.g2o"))
        self.assertFalse(any(self._is_edge(l, _A[1], _A[3]) for l in plain),
            "Low-association LC should have been filtered out")

    def test_threshold_drops_low_association_lc_in_marked_mode(self):
        """LC2 (2 associations) must be absent when thresh=3 in marked mode."""
        marked = self._call(add_lc_comment_markers=True, thresh=3,
                            out=os.path.join(self.tmp, "out_thresh_marked.g2o"))
        self.assertFalse(any(self._is_edge(l, _A[1], _A[3]) for l in marked),
            "Low-association LC should have been filtered out")

    def test_threshold_keeps_high_association_lc(self):
        """LC1 (5 associations) must be present when thresh=3."""
        plain = self._call(add_lc_comment_markers=False, thresh=3,
                           out=os.path.join(self.tmp, "out_thresh_keep.g2o"))
        self.assertTrue(any(self._is_edge(l, _A[0], _A[2]) for l in plain),
            "High-association LC should have been kept")


if __name__ == "__main__":
    unittest.main()
