import os
import tempfile
import unittest

import gtsam

from roman.offline_rpgo.combine_loop_closures import combine_loop_closures

# GTSAM symbol values for robots 'a' and 'b', keyframes 0-4
_A = [gtsam.symbol('a', i) for i in range(5)]
_B = [gtsam.symbol('b', i) for i in range(5)]

# Minimal upper-triangle of a 6x6 identity information matrix (21 values)
_INFO = "1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1"


def _edge(v0, v1, tx=1.0):
    return f"EDGE_SE3:QUAT {v0} {v1} {tx} 0 0 0 0 0 1 {_INFO}"


def _vertex(sym, x=0.0):
    return f"VERTEX_SE3:QUAT {sym} {x} 0 0 0 0 0 1"


class TestCombineLoopClosuresLcMarkers(unittest.TestCase):
    """
    Verify that add_lc_comment_markers=True produces output identical to
    add_lc_comment_markers=False except for the inserted "# LC:" lines.

    Setup:
      Sparse graph: robot 'a', 5 keyframes (idx 0-4) at t = 0, 5, 10, 15, 20 s.
      Dense graph:  robot 'a', 5 keyframes (idx 0-4) at t = 0, 5, 10, 15, 20 s.
                    Two LC edges:
                      LC1: dense kf1 -> dense kf3  (t=5s  -> t=15s, |idx|=2)
                           extract_additional_lc strips t=0 from sparse, so:
                             kf1 (t=5s)  -> nearest sparse = kf1 (t=5s)  -> _A[1]
                             kf3 (t=15s) -> nearest sparse = kf3 (t=15s) -> _A[3]
                           Remaps to _A[1] -> _A[3]  (non-adjacent, diff=2)
                      LC2: dense kf0 -> dense kf2  (t=0s  -> t=10s, |idx|=2)
                             kf0 (t=0s)  -> nearest sparse after stripping t=0 = kf1 (t=5s) -> _A[1]
                             kf2 (t=10s) -> nearest sparse = kf2 (t=10s) -> _A[2]
                           Remaps to _A[1] -> _A[2]  (adjacent, diff=1)
                           This is the previously-dropped case: |idx0-idx1|==1 while being a real LC.
    """

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

        # Sparse: 5 kf at t = 0, 5, 10, 15, 20 s
        sparse_g2o = "\n".join(
            [_vertex(_A[i], x=float(i * 2)) for i in range(5)]
            + [_edge(_A[i], _A[i + 1]) for i in range(4)]
        ) + "\n"

        sparse_times = "\n".join(
            [f"0 {i} {int(i * 5 * 1e9)} xxx" for i in range(5)]
        ) + "\n"

        # Dense: same times, 4 odometry edges, and the two LCs described above.
        dense_g2o = "\n".join(
            [_vertex(_A[i], x=float(i)) for i in range(5)]
            + [_edge(_A[i], _A[i + 1]) for i in range(4)]
            + [
                "# LC: 5",
                _edge(_A[1], _A[3], tx=2.0),   # LC1 -> remaps to _A[1] -> _A[3]
                "# LC: 4",
                _edge(_A[0], _A[2], tx=2.0),   # LC2 -> remaps to _A[1] -> _A[2] (adjacent sparse)
            ]
        ) + "\n"

        dense_times = "\n".join(
            [f"0 {i} {int(i * 5 * 1e9)} xxx" for i in range(5)]
        ) + "\n"

        def write(name, content):
            path = os.path.join(self.tmp, name)
            with open(path, "w") as f:
                f.write(content)
            return path

        self.sparse_g2o   = write("sparse.g2o",      sparse_g2o)
        self.sparse_times = write("sparse.time.txt", sparse_times)
        self.dense_g2o    = write("dense.g2o",       dense_g2o)
        self.dense_times  = write("dense.time.txt",  dense_times)

        # Ground-truth LC vertex pairs after sparse remapping (verified against actual output)
        self.expected_lc_pairs = [
            (_A[1], _A[3]),   # LC1: non-adjacent sparse indices
            (_A[1], _A[2]),   # LC2: adjacent sparse indices (the previously-dropped case)
        ]

    def _call(self, add_lc_comment_markers: bool) -> list:
        out = os.path.join(
            self.tmp,
            f"out_{'marked' if add_lc_comment_markers else 'plain'}.g2o",
        )
        return combine_loop_closures(
            g2o_reference=self.sparse_g2o,
            g2o_extra_lc=self.dense_g2o,
            vertex_times_reference=self.sparse_times,
            vertex_times_extra_lc=self.dense_times,
            output_file=out,
            add_lc_comment_markers=add_lc_comment_markers,
        )

    def test_marked_identical_except_lc_comment_lines(self):
        """Removing all '# LC:' lines from the marked output must equal the plain output."""
        plain_lines  = self._call(add_lc_comment_markers=False)
        marked_lines = self._call(add_lc_comment_markers=True)
        stripped = [l for l in marked_lines if l.strip() != "# LC:"]
        self.assertEqual(plain_lines, stripped)

    def test_marked_lc_edges_preceded_by_lc_comment(self):
        """Every known LC edge — including the adjacent-index one — must be preceded by '# LC:'."""
        marked_lines = self._call(add_lc_comment_markers=True)
        stripped_lines = [l.strip() for l in marked_lines]

        # Only search in the new-LC section to avoid matching same-vertex odometry edges
        # that exist in the sparse reference graph.
        try:
            lc_section_start = stripped_lines.index("# NEW LOOP CLOSURES") + 1
        except ValueError:
            self.fail("'# NEW LOOP CLOSURES' marker not found in output")
        lc_section = stripped_lines[lc_section_start:]

        for v0, v1 in self.expected_lc_pairs:
            prefix = f"EDGE_SE3:QUAT {v0} {v1}"
            for i, line in enumerate(lc_section):
                if line.startswith(prefix):
                    prev = lc_section[i - 1] if i > 0 else ""
                    self.assertEqual(
                        prev, "# LC:",
                        msg=f"LC edge '{prefix}' not preceded by '# LC:'",
                    )
                    break
            else:
                self.fail(f"Expected LC edge '{prefix}' not found in LC section")

    def test_plain_has_no_lc_comment_lines(self):
        """Plain output must contain no '# LC:' lines."""
        plain_lines = self._call(add_lc_comment_markers=False)
        lc_comments = [l for l in plain_lines if l.strip() == "# LC:"]
        self.assertEqual(lc_comments, [])


class TestCombineLoopClosuresAdjacentDenseLc(unittest.TestCase):
    """
    Regression test: a dense g2o LC edge whose vertex indices differ by exactly 1
    (|idx|==1) but is preceded by '# LC:' must NOT be dropped during extraction
    (step 2c).  The old code used np.abs(vertex0-vertex1)==1 as an unconditional
    odometry filter, so these edges were silently lost.

    Setup:
      Sparse: robot 'a', 5 kf at t = 0, 5, 10, 15, 20 s.
      Dense:  robot 'a', 5 kf at t = 0, 5, 10, 15, 20 s.
              One LC edge: dense kf3 -> dense kf4  (t=15s -> t=20s, |idx|=1)
              preceded by '# LC:'.
              Remaps to _A[3] -> _A[4] in sparse (still adjacent, but it IS a
              real LC and must not be dropped).
    """

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

        sparse_g2o = "\n".join(
            [_vertex(_A[i], x=float(i * 2)) for i in range(5)]
            + [_edge(_A[i], _A[i + 1]) for i in range(4)]
        ) + "\n"

        sparse_times = "\n".join(
            [f"0 {i} {int(i * 5 * 1e9)} xxx" for i in range(5)]
        ) + "\n"

        dense_g2o = "\n".join(
            [_vertex(_A[i], x=float(i)) for i in range(5)]
            + [_edge(_A[i], _A[i + 1]) for i in range(4)]
            + [
                "# LC: 3",
                _edge(_A[3], _A[4], tx=3.0),  # |idx|==1, must not be dropped
            ]
        ) + "\n"

        dense_times = "\n".join(
            [f"0 {i} {int(i * 5 * 1e9)} xxx" for i in range(5)]
        ) + "\n"

        def write(name, content):
            path = os.path.join(self.tmp, name)
            with open(path, "w") as f:
                f.write(content)
            return path

        self.sparse_g2o   = write("sparse.g2o",      sparse_g2o)
        self.sparse_times = write("sparse.time.txt", sparse_times)
        self.dense_g2o    = write("dense.g2o",       dense_g2o)
        self.dense_times  = write("dense.time.txt",  dense_times)

    def _call(self, add_lc_comment_markers: bool) -> list:
        out = os.path.join(self.tmp, "out.g2o")
        return combine_loop_closures(
            g2o_reference=self.sparse_g2o,
            g2o_extra_lc=self.dense_g2o,
            vertex_times_reference=self.sparse_times,
            vertex_times_extra_lc=self.dense_times,
            output_file=out,
            add_lc_comment_markers=add_lc_comment_markers,
        )

    def _lc_section(self, lines: list) -> list:
        stripped = [l.strip() for l in lines]
        try:
            start = stripped.index("# NEW LOOP CLOSURES") + 1
        except ValueError:
            self.fail("'# NEW LOOP CLOSURES' marker not found in output")
        return stripped[start:]

    def test_adjacent_dense_lc_not_dropped(self):
        """LC edge with |idx|==1 in dense g2o must appear in the output after remapping."""
        lc_section = self._lc_section(self._call(add_lc_comment_markers=False))
        prefix = f"EDGE_SE3:QUAT {_A[3]} {_A[4]}"
        self.assertTrue(
            any(l.startswith(prefix) for l in lc_section),
            f"Expected LC edge '{prefix}' not found — was it silently dropped?",
        )

    def test_adjacent_dense_lc_preceded_by_marker(self):
        """When markers are on, the adjacent-index LC must be preceded by '# LC:'."""
        lc_section = self._lc_section(self._call(add_lc_comment_markers=True))
        prefix = f"EDGE_SE3:QUAT {_A[3]} {_A[4]}"
        for i, line in enumerate(lc_section):
            if line.startswith(prefix):
                prev = lc_section[i - 1] if i > 0 else ""
                self.assertEqual(prev, "# LC:",
                    msg=f"LC edge '{prefix}' not preceded by '# LC:'")
                return
        self.fail(f"Expected LC edge '{prefix}' not found in LC section")


class TestCombineLoopClosuresSelfLoopFilter(unittest.TestCase):
    """
    Verify that self-loops (both LC endpoints remapping to the same sparse vertex)
    are filtered out, while inter-robot LCs whose endpoints share the same keyframe
    index but belong to different robots are never filtered.

    Root cause in practice: the robot is stationary for a long stretch at the start,
    so all dense vertices in that window remap to the same sparse keyframe after
    extract_additional_lc strips kf0.
    """

    def _write(self, tmp, name, content):
        path = os.path.join(tmp, name)
        with open(path, "w") as f:
            f.write(content)
        return path

    def _lc_section(self, lines):
        stripped = [l.strip() for l in lines]
        try:
            start = stripped.index("# NEW LOOP CLOSURES") + 1
        except ValueError:
            self.fail("'# NEW LOOP CLOSURES' marker not found in output")
        return stripped[start:]

    def test_single_robot_self_loop_filtered(self):
        """
        Single-robot LC whose both endpoints remap to the same sparse keyframe
        must be dropped.

        Sparse: robot 'a', kf0 at t=0s, kf1 at t=300s.
        Dense:  robot 'a', kf0 at t=0s, kf1 at t=100s, kf2 at t=200s.
                LC: dense kf0 (t=0) -> dense kf2 (t=200).
        extract_additional_lc strips kf0 from sparse, leaving only kf1 (t=300s).
        nearest_time for both t=0 and t=200 is 300s -> both remap to symbol('a',1).
        vxs_ref[0] == vxs_ref[1] -> self-loop filter -> 0 LCs in output.
        """
        tmp = tempfile.mkdtemp()

        sparse_g2o = "\n".join([
            _vertex(_A[0], x=0.0),
            _vertex(_A[1], x=10.0),
            _edge(_A[0], _A[1], tx=10.0),
        ]) + "\n"
        sparse_times = "0 0 0 xxx\n0 1 300000000000 xxx\n"

        dense_g2o = "\n".join([
            _vertex(_A[0], x=0.0),
            _vertex(_A[1], x=5.0),
            _vertex(_A[2], x=10.0),
            _edge(_A[0], _A[1], tx=5.0),
            _edge(_A[1], _A[2], tx=5.0),
            "# LC:",
            _edge(_A[0], _A[2], tx=10.0),
        ]) + "\n"
        dense_times = "0 0 0 xxx\n0 1 100000000000 xxx\n0 2 200000000000 xxx\n"

        lines = combine_loop_closures(
            g2o_reference=self._write(tmp, "sparse.g2o", sparse_g2o),
            g2o_extra_lc=self._write(tmp, "dense.g2o", dense_g2o),
            vertex_times_reference=self._write(tmp, "sparse.time.txt", sparse_times),
            vertex_times_extra_lc=self._write(tmp, "dense.time.txt", dense_times),
            output_file=os.path.join(tmp, "out.g2o"),
        )

        lc_edges = [l for l in self._lc_section(lines) if l.startswith("EDGE_SE3:QUAT")]
        self.assertEqual(
            len(lc_edges), 0,
            msg=f"Expected 0 LCs after self-loop filter, got {len(lc_edges)}: {lc_edges}",
        )

    def test_inter_robot_lc_not_filtered(self):
        """
        Inter-robot LC whose endpoints remap to symbol('a',1) and symbol('b',1)
        must NOT be filtered — different robots means vxs_ref[0] != vxs_ref[1].

        Sparse: robots 'a' and 'b', each kf0 at t=0s and kf1 at t=300s.
        Dense:  same structure; inter-robot LC between _A[0] (t=0) and _B[0] (t=0).
        Both endpoints remap to index 1 in their respective robots:
          symbol('a',1) != symbol('b',1) -> not a self-loop -> kept -> 1 LC in output.
        """
        tmp = tempfile.mkdtemp()

        sparse_g2o = "\n".join([
            _vertex(_A[0], x=0.0),
            _vertex(_A[1], x=10.0),
            _edge(_A[0], _A[1], tx=10.0),
            _vertex(_B[0], x=0.0),
            _vertex(_B[1], x=10.0),
            _edge(_B[0], _B[1], tx=10.0),
        ]) + "\n"
        sparse_times = (
            "0 0 0 xxx\n0 1 300000000000 xxx\n"
            "1 0 0 xxx\n1 1 300000000000 xxx\n"
        )

        dense_g2o = "\n".join([
            _vertex(_A[0], x=0.0),
            _vertex(_A[1], x=5.0),
            _vertex(_A[2], x=10.0),
            _edge(_A[0], _A[1], tx=5.0),
            _edge(_A[1], _A[2], tx=5.0),
            _vertex(_B[0], x=0.0),
            _vertex(_B[1], x=5.0),
            _vertex(_B[2], x=10.0),
            _edge(_B[0], _B[1], tx=5.0),
            _edge(_B[1], _B[2], tx=5.0),
            "# LC:",
            _edge(_A[0], _B[0], tx=0.0),  # identity: both at same location
        ]) + "\n"
        dense_times = (
            "0 0 0 xxx\n0 1 100000000000 xxx\n0 2 200000000000 xxx\n"
            "1 0 0 xxx\n1 1 100000000000 xxx\n1 2 200000000000 xxx\n"
        )

        lines = combine_loop_closures(
            g2o_reference=self._write(tmp, "sparse.g2o", sparse_g2o),
            g2o_extra_lc=self._write(tmp, "dense.g2o", dense_g2o),
            vertex_times_reference=self._write(tmp, "sparse.time.txt", sparse_times),
            vertex_times_extra_lc=self._write(tmp, "dense.time.txt", dense_times),
            output_file=os.path.join(tmp, "out.g2o"),
        )

        lc_edges = [l for l in self._lc_section(lines) if l.startswith("EDGE_SE3:QUAT")]
        self.assertEqual(
            len(lc_edges), 1,
            msg=f"Expected 1 inter-robot LC (not filtered), got {len(lc_edges)}: {lc_edges}",
        )
        expected_prefix = f"EDGE_SE3:QUAT {_A[1]} {_B[1]}"
        self.assertTrue(
            any(l.startswith(expected_prefix) for l in lc_edges),
            msg=f"Expected LC '{expected_prefix}', got: {lc_edges}",
        )


if __name__ == "__main__":
    unittest.main()
