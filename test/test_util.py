import unittest

import sys
sys.stderr = sys.stdout

import catkin_lint.util as util

class UtilTest(unittest.TestCase):
    def test_word_split(self):
        result = util.word_split("CamelCase")
        self.assertEqual([ "camel", "case" ], result)
        result = util.word_split("HTTPConnector")
        self.assertEqual([ "http", "connector" ], result)
        result = util.word_split("c_style_identifier")
        self.assertEqual([ "c", "style", "identifier" ], result)
        result = util.word_split("OpenSSL")
        self.assertEqual([ "open", "ssl" ], result)
        result = util.word_split("OGRE")
        self.assertEqual([ "ogre" ], result)
        result = util.word_split("getPS2Port")
        self.assertEqual([ "get", "ps2", "port" ], result)
        result = util.word_split("2BeOrNot2b")
        self.assertEqual([ "2", "be", "or", "not2b" ], result)
        result = util.word_split("C-3PO")
        self.assertEqual([ "c", "3", "po" ], result)
        result = util.word_split("c-3po")
        self.assertEqual([ "c", "3po" ], result)
