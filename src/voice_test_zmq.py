# -*- coding: utf-8 -*-
#! /usr/bin/env python

import zmq

if __name__ == "__main__":
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect('tcp://localhost:5557')
    socket.send_string('abc')
    print(socket.recv_string(0))