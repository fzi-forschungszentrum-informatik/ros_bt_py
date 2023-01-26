Development Notes
=================

This section describes development notes and remarks from the developers about design decisions.

AsyncServiceProxy
-----------------

To effectively use ServiceProxies with BehaviorTrees a method to call ROS ServiceProxies
asynchronously is required.
During the initial implementation of the library a solution using Process forking was implemented.
This is implemented in the ``AsyncServiceProxy`` class.
Each time a service is called, the ``AsyncServiceProxy`` aiming to call a service, spawns a new
Process that handles the Service call.
To ensure data consistency between the Process running the ``AsyncServiceProxy`` and the spawned
Process a Python ``SyncManager`` was used.

A downside of this implementation was that for each Service call a number of processes and IPC
channels are created.
When working with large trees this could lead to ``Too many open files`` errors preventing the
tree from running.

Thus, a version using ``Threads`` was implemented in place of ``Processes``.
While a ``Thread``-based solution is limited in parts by the `GIL <https://wiki.python.org/moin/GlobalInterpreterLock>`_,
the limitations are far less severe compared to the OS resource issues the ``Processes`` solution
expressed.
Data is now synchronized using shared memory and a mutex.
Additionally a singleton pattern is implemented to allow the sharing of ``ServiceProxy``-Instances
between ``AsyncServiceProxy`` for the same service URL.
All created ``ServiceProxy`` instances are stored in a global dict, organized by their associated
service url and type, and can be claimed by a ``AsyncServiceProxy`` for a call.
Importantly, they are unclaimed after each call to make them available to other ``AsyncServiceProxy`` instances.
Should two proxies aim to call the same service simultaneously, a new ``ServiceProxy`` is created on-demand, thus
reducing the overall resource footprint further.
As the newly created proxy is not destroyed, on further occurrences no new ``ServiceProxy`` must be allocated.
