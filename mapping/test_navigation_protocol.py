"""使用赛务提供的独立参考向量验证协议编码。"""

import unittest

from navigation_protocol import crc8, crc16, next_seq, pack_command


class ProtocolTests(unittest.TestCase):
    def test_crc_reference_values(self):
        self.assertEqual(crc8(b"123456789"), 0x0B)
        self.assertEqual(crc16(b"123456789"), 0x6F91)

    def test_reference_frames_with_corrected_stop_example(self):
        vectors = [
            ((1, 0, 0), "A5 0C 00 01 26 00 00 80 3F 00 00 00 00 00 00 00 00 6B A3"),
            # 文档停车示例多写一个00；此处按19字节结构修正。
            ((0, 0, 0), "A5 0C 00 01 26 00 00 00 00 00 00 00 00 00 00 00 00 18 9F"),
            ((1, -1, 3), "A5 0C 00 01 26 00 00 80 3F 00 00 80 BF 00 00 40 40 D3 96"),
        ]
        for velocities, expected in vectors:
            with self.subTest(velocities=velocities):
                actual = pack_command(*velocities, seq=1)
                self.assertEqual(len(actual), 19)
                self.assertEqual(actual, bytes.fromhex(expected))

    def test_nonfinite_values_rejected(self):
        for value in (float("nan"), float("inf"), float("-inf")):
            for axis in range(3):
                values = [0, 0, 0]
                values[axis] = value
                with self.subTest(value=value, axis=axis):
                    with self.assertRaises(ValueError):
                        pack_command(*values)

    def test_float32_overflow_rejected(self):
        with self.assertRaises(ValueError):
            pack_command(1e100, 0, 0)

    def test_invalid_sequence_rejected(self):
        for value in (-1, 256, 1.5, True):
            with self.subTest(seq=value):
                with self.assertRaises(ValueError):
                    pack_command(0, 0, 0, value)

    def test_sequence_wrap(self):
        self.assertEqual(next_seq(1), 2)
        self.assertEqual(next_seq(255), 0)
        self.assertEqual(pack_command(0, 0, 0, seq=0)[3], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
