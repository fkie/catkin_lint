import unittest
from .helper import patch
import catkin_lint.util as util
import tempfile
import shutil
import os


def force_fail(*args, **kwargs):
    raise OSError("Mock fail")


class UtilTest(unittest.TestCase):
    def test_word_split(self):
        """Test word_split() utility function"""
        result = util.word_split("CamelCase")
        self.assertEqual(["camel", "case"], result)
        result = util.word_split("HTTPConnector")
        self.assertEqual(["http", "connector"], result)
        result = util.word_split("c_style_identifier")
        self.assertEqual(["c", "style", "identifier"], result)
        result = util.word_split("OpenSSL")
        self.assertEqual(["open", "ssl"], result)
        result = util.word_split("OGRE")
        self.assertEqual(["ogre"], result)
        result = util.word_split("getPS2Port")
        self.assertEqual(["get", "ps2", "port"], result)
        result = util.word_split("2BeOrNot2b")
        self.assertEqual(["2", "be", "or", "not2b"], result)
        result = util.word_split("C-3PO")
        self.assertEqual(["c", "3", "po"], result)
        result = util.word_split("c-3po")
        self.assertEqual(["c", "3po"], result)

    def test_is_sorted(self):
        """Test is_sorted() utility function"""
        self.assertTrue(util.is_sorted(["a", "b", "c", "d"]))
        self.assertFalse(util.is_sorted(["b", "a", "c", "d"]))
        self.assertFalse(util.is_sorted(["a", "c", "b", "d"]))
        self.assertFalse(util.is_sorted(["a", "b", "d", "c"]))

    def test_write_atomic(self):
        """Test write_atomic() utility function"""
        tmpdir = tempfile.mkdtemp()
        try:
            with patch("os.unlink", force_fail):
                with patch("os.rename", force_fail):
                    self.assertRaises(OSError, util.write_atomic, os.path.join(tmpdir, "test"), b"test")
                    self.assertFalse(os.path.exists(os.path.join(tmpdir, "test")))
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)
