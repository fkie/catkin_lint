#!/usr/bin/env python
"""
Copyright (c) 2013-2015 Fraunhofer FKIE

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of the Fraunhofer organization nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

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

class Rosdep(object):

    def __init__(self, view=None):
        self.view = view

    def is_ros(self, name):
        if self.view is None: return False
        try:
            return self.view.lookup(name).data["_is_ros"]
        except KeyError:
            return False

    def is_pkg(self, name):
        if self.view is None: return False
        return name in self.view.keys()

    def ok(self):
        return self.view is not None


def get_rosdep():
    from rosdep2.lookup import RosdepLookup
    from rosdep2.rospkg_loader import DEFAULT_VIEW_KEY
    from rosdep2.sources_list import SourcesListLoader
    sources_loader = SourcesListLoader.create_default()
    lookup = RosdepLookup.create_from_rospkg(sources_loader=sources_loader)
    return Rosdep(view=lookup.get_rosdep_view(DEFAULT_VIEW_KEY))
