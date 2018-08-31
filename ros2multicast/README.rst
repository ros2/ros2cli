ros2multicast
=============

The package provides the multicast command for the ROS 2 command line tools.

The tool can be used to check if multicast UDP packets are passed between two endpoints.

First invoke the following command on one machine:

.. code-block:: bash

    $ ros2 multicast receive

While the first machine is waiting for a packet to arrive invoke the following command on another machine:

.. code-block:: bash

    $ ros2 multicast send

When successful the first machine will output the received message "Hello World!".
