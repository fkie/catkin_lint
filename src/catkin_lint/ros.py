# coding=utf-8
#
# catkin_lint
# Copyright (c) 2013-2018 Fraunhofer FKIE
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of the Fraunhofer organization nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
# IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import sys
from catkin_pkg.package import parse_package_string


class DummyRospkg(object):

    def list(self):
        return []


class Rosdep(object):

    def __init__(self, view=None, quiet=False):
        self.view = view
        self.quiet = quiet

    def is_ros(self, name):
        try:
            if self.view is not None:
                return self.view.lookup(name).data["_is_ros"]
        except KeyError:
            pass
        return False

    def has_key(self, name):
        return self.view is not None and name in self.view.keys()


def get_rosdep(quiet):
    from rosdep2.lookup import RosdepLookup
    from rosdep2.rospkg_loader import DEFAULT_VIEW_KEY
    from rosdep2.sources_list import SourcesListLoader
    dummy = DummyRospkg()
    sources_loader = SourcesListLoader.create_default()
    lookup = RosdepLookup.create_from_rospkg(rospack=dummy, rosstack=dummy, sources_loader=sources_loader)
    return Rosdep(view=lookup.get_rosdep_view(DEFAULT_VIEW_KEY), quiet=quiet)


class Rosdistro(object):
    def __init__(self, dist=None, quiet=False):
        self.dist = dist
        self.quiet = quiet

    def ok(self):
        return self.dist is not None

    def download_manifest(self, name):
        if not self.quiet:
            sys.stderr.write("catkin_lint: downloading package manifest for '%s'\n" % name)
        package_xml = self.dist.get_release_package_xml(name)
        return parse_package_string(package_xml) if package_xml else None


_rosdistro_cache = {}


def get_rosdistro(quiet):
    global _rosdistro_cache
    dist = None
    if "ROS_DISTRO" in os.environ:
        distro_id = os.environ["ROS_DISTRO"]
        if distro_id not in _rosdistro_cache:
            try:
                from rosdistro import get_index, get_index_url, get_cached_distribution
                url = get_index_url()
                if not quiet:
                    sys.stderr.write("catkin_lint: downloading %s package index from %s\n" % (distro_id, url))
                index = get_index(url)
                dist = get_cached_distribution(index, distro_id, allow_lazy_load=True)
            except Exception as err:
                if not quiet:
                    sys.stderr.write("catkin_lint: cannot initialize rosdistro: %s\n" % str(err))
            _rosdistro_cache[distro_id] = dist
        dist = _rosdistro_cache[distro_id]
    return Rosdistro(dist=dist, quiet=quiet)
