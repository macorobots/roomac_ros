#!/usr/bin/env python

# Copyright (c) 2018, G.A. vd. Hoorn
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# https://github.com/gavanderhoorn/rosbag_fixer
# rosrun roomac_rtabmap fix_bag_msg_def.py --use-local-defs in.bag out.bag

import argparse
import os
import sys

try:
    import roslib.message
except:
    sys.stderr.write("Could not import 'roslib', make sure it is installed, "
        "and make sure you have sourced the ROS environment setup file if "
        "necessary.\n\n")
    sys.exit(1)

try:
    import rosbag
except:
    sys.stderr.write("Could not import 'rosbag', make sure it is installed, "
        "and make sure you have sourced the ROS environment setup file if "
        "necessary.\n\n")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', action='store_true', help='Be verbose')
    parser.add_argument('-l', '--use-local-defs', dest='use_local', action='store_true', help='Use message defs from local system (as opposed to reading them from the provided mappings)')
    parser.add_argument('-c', '--callerid', type=str, help='Callerid (ie: publisher)')
    parser.add_argument('-m', '--map', dest='mappings', type=str, nargs=1, action='append', help='Mapping topic type -> good msg def (multiple allowed)', default=[])
    parser.add_argument('inbag', help='Input bagfile')
    parser.add_argument('outbag', help='Output bagfile')
    args = parser.parse_args()

    if not os.path.isfile(args.inbag):
        sys.stderr.write('Cannot locate input bag file [%s]\n' % args.inbag)
        sys.exit(os.EX_USAGE)

    if os.path.realpath(args.inbag) == os.path.realpath(args.outbag):
        sys.stderr.write('Cannot use same file as input and output [%s]\n' % args.inbag)
        sys.exit(os.EX_USAGE)

    if len(args.mappings) > 0 and args.use_local:
        sys.stderr.write("Cannot use both mappings and local defs.\n")
        sys.exit(os.EX_USAGE)


    # TODO: make this nicer. Figure out the complete msg text without relying on external files
    msg_def_maps = {}
    if len(args.mappings) > 0:
        print ("Mappings provided:")
        for mapping in args.mappings:
            map_msg, map_file = mapping[0].split(':')
            print ("  {:40s}: {}".format(map_msg, map_file))

            # 'geometry_msgs/PoseStamped:geometry_msgs_pose_stamped_good.txt'
            with open(map_file, 'r') as f:
                new_def = f.read()
                # skip first line, it contains something like '[geometry_msgs/PoseStamped]:'
                msg_def_maps[map_msg] = new_def.split('\n', 1)[1]
                #print (msg_def_maps[map_msg])

    else:
        if not args.use_local:
            print ("No mappings provided and not allowed to use local msg defs. "
                   "That is ok, but this won't fix anything like this.")

    print ("")


    # open bag to fix
    bag = rosbag.Bag(args.inbag)

    # filter for all connections that pass the filter expression
    # if no 'callerid' specified, returns all connections
    conxs = bag._get_connections(connection_filter=
        lambda topic, datatype, md5sum, msg_def, header:
            header['callerid'] == args.callerid if args.callerid else True)

    conxs = list(conxs)

    if not conxs:
        print ("No topics found for callerid '{}'. Make sure it is correct.\n".format(args.callerid))
        sys.exit(1)


    def_replaced = []
    def_not_found = []
    def_not_replaced = []

    # loop over connections, find out which msg type they use and replace
    # msg defs if needed. Note: this is a rather primitive way to approach
    # this and absolutely not guaranteed to work.
    # It does work for me though ..
    for conx in conxs:
        # see if we have a mapping for that
        msg_type = conx.datatype
        if not msg_type in msg_def_maps:
            if not args.use_local:
                def_not_found.append((conx.topic, msg_type))
                continue

            # don't have mapping, but are allowed to use local msg def: retrieve
            # TODO: properly deal with get_message_class failing
            sys_class = roslib.message.get_message_class(msg_type)
            if sys_class is None:
                raise ValueError("Message class '" + msg_type + "' not found.")
            msg_def_maps[conx.datatype] = sys_class._full_text

        # here, we either already had a mapping or one was just created
        full_msg_text = msg_def_maps[msg_type]

        # don't touch anything if not needed (note: primitive check)
        if conx.header['message_definition'] == full_msg_text:
            def_not_replaced.append((conx.topic, msg_type))
            continue

        # here we really should replace the msg def, so do it
        conx.header['message_definition'] = full_msg_text
        conx.msg_def = full_msg_text

        def_replaced.append((conx.topic, msg_type))


    # print stats
    if def_replaced and args.verbose:
        print ("Replaced {} message definition(s):".format(len(def_replaced)))
        for topic, mdef in def_replaced:
            print ("  {:40s} : {}".format(mdef, topic))
        print ("")

    if def_not_replaced and args.verbose:
        print ("Skipped {} message definition(s) (already ok):".format(len(def_not_replaced)))
        for topic, mdef in def_not_replaced:
            print ("  {:40s} : {}".format(mdef, topic))
        print ("")

    if def_not_found and args.verbose:
        print ("Could not find {} message definition(s):".format(len(def_not_found)))
        for topic, mdef in def_not_found:
            print ("  {:40s} : {}".format(mdef, topic))
        print ("")



    print ("Writing out fixed bag ..")

    # write result to new bag
    # TODO: can this be done more efficiently? We only changed the connection infos.
    with rosbag.Bag(args.outbag, 'w') as outbag:
        # shamelessly copied from Rosbag itself
        meter = rosbag.rosbag_main.ProgressMeter(outbag.filename, bag._uncompressed_size)
        total_bytes = 0
        for topic, raw_msg, t in bag.read_messages(raw=True):
            msg_type, serialized_bytes, md5sum, pos, pytype = raw_msg

            outbag.write(topic, raw_msg, t, raw=True)

            total_bytes += len(serialized_bytes)
            meter.step(total_bytes)

        meter.finish()

    print ("\ndone")
    print ("\nThe new bag probably needs to be re-indexed. Use 'rosbag reindex {}' for that.\n".format(outbag.filename))


if __name__ == '__main__':
    main()