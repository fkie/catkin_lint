#!/usr/bin/env python
"""
Copyright (c) 2013 Fraunhofer FKIE

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
from catkin_lint.main import ERROR, WARNING, NOTICE
import re

def package_description(linter):
    # Check for meaningless package descriptions
    buzzwords = [
        r"(an)?other",
        r"(python\s+|c(\+\+)?\s+|java\s+)?code(\s+snippets?)?",
        r"(ros\s+)?nodes?",
        r"\d+",
        r"a\s+few",
        r"add(s|ed)?",
        r"all",
        r"an?",
        r"and",
        r"are",
        r"be",
        r"boilerplate",
        r"both",
        r"can",
        r"comprise(s|d)?",
        r"contain(s|ed)?",
        r"describe(s|d)?",
        r"descriptions?",
        r"did",
        r"different",
        r"do(es)?",
        r"done",
        r"examples?",
        r"executables?",
        r"for",
        r"from",
        r"functionalit(y|ies)",
        r"had",
        r"has",
        r"have",
        r"implementations?",
        r"in",
        r"include(s|d)?",
        r"interfaces?",
        r"into",
        r"is",
        r"librar(y|ies)",
        r"meaningless",
        r"miscellaneous",
        r"multiple",
        r"no(thing|ne|t)?",
        r"of",
        r"offer(s|ed)",
        r"one",
        r"or",
        r"packages?",
        r"programs?",
        r"provide(s|d)?",
        r"purpose",
        r"routines?",
        r"runs?",
        r"sets?",
        r"several",
        r"some",
        r"suppl(y|ies|ied)",
        r"that",
        r"the",
        r"these",
        r"this",
        r"those",
        r"three",
        r"to",
        r"todo",
        r"tools?",
        r"two",
        r"use[sd]?",
        r"useful",
        r"various",
        r"versions?",
        r"which",
        r"with",
        r"work\s+in\s+progress",
    ]
    def on_init(info):
        chatter = re.match(r"((\s|[.,!?;()/])*\b(%s|%s)\b)+" % (info.manifest.name, r"|".join(buzzwords)), info.manifest.description, re.IGNORECASE)
        if chatter is not None:
            s = chatter.group(0).replace("\n", " ").strip()
            if info.manifest.description[chatter.end(0):].strip():
                if len(s.split()) > 1:
                    info.report(NOTICE, "DESCRIPTION_BOILERPLATE", text=s)
            else:
                info.report(NOTICE, "DESCRIPTION_MEANINGLESS", text=s)

    linter.add_init_hook(on_init)
