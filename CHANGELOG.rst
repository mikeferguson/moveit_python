^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.5 (2023-01-03)
------------------
* fix CI after adding setup tools dependency
* Contributors: Michael Ferguson

0.4.4 (2022-12-23)
------------------
* use setuptools (`#33 <https://github.com/mikeferguson/moveit_python/issues/33>`_)
  setup from distutils is deprecated and will be removed eventually.
  It already breaks on Debian testing.
* Contributors: Michael Görner

0.4.3 (2022-01-14)
------------------
* fix: Add `attached_object_touch_links` to pickup (`#31 <https://github.com/mikeferguson/moveit_python/issues/31>`_)
* Contributors: Kai Waelti

0.4.2 (2021-06-28)
------------------
* New cone primitive and box primitive frame fix (`#30 <https://github.com/mikeferguson/moveit_python/issues/30>`_)
  * Box primitive frame independent of planning scene
  * Added cone solid primitive
* Contributors: Kiran Prasad

0.4.1 (2021-02-11)
------------------
* add support for objects not in fixed_frame
* Contributors: Michael Ferguson

0.4.0 (2021-01-17)
------------------
* various fixes for noetic/python3 (`#28 <https://github.com/mikeferguson/moveit_python/issues/28>`_)
* Contributors: Michael Ferguson

0.3.6 (2020-12-25)
------------------
* update package.xml for python3 support
* Contributors: Michael Ferguson

0.3.5 (2020-12-25)
------------------
* use "is None"
* Fixed a comparison and logical statement order issue. (`#26 <https://github.com/mikeferguson/moveit_python/issues/26>`_)
* fix pyassimp indices bug (`#27 <https://github.com/mikeferguson/moveit_python/issues/27>`_)
* Contributors: Karl Kangur, Michael Ferguson, Shingo Kitagawa

0.3.4 (2020-04-22)
------------------
* Fixed error message when removing attached object. (`#25 <https://github.com/mikeferguson/moveit_python/issues/25>`_)
  Co-authored-by: Karl Kangur <karl.kangur@exwzd.com>
* Merge pull request `#24 <https://github.com/mikeferguson/moveit_python/issues/24>`_ from v4hn/pr-master-python3
  python3 compatible API & syntax
* python3 compatible API & syntax
* Contributors: Karl Kangur, Michael Ferguson, v4hn

0.3.3 (2019-11-21)
------------------
* add sphere to planning scene interface
* add move_group parameter to MoveGroup class
* Contributors: Michael Ferguson, arwtyxouymz

0.3.2 (2019-11-14)
------------------
* enable pyassimp for melodic
* update maintainer email
* Contributors: Michael Ferguson, Shingo Kitagawa

0.3.1 (2018-08-13)
------------------
* add missing namespace to apply service
* Contributors: Michael Ferguson

0.3.0 (2018-07-31)
------------------
* add namespace functionality on planning_scene_interface.py
* additional cleanup/documentation
  * use apply_service for colors
  * rename sentUpdate to sendUpdate, add docs
  * rename wait param to use_service (the meaning has changed)
  * remove some spammy logging
  * don't waitForSync when using service
* Merge pull request `#10 <https://github.com/mikeferguson/moveit_python/issues/10>`_ from alemme/master
  adapt the code for the `apply` service
* added services to add objects to environment and attach them. Following http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/planning_scene_ros_api_tutorial.html#interlude-synchronous-vs-asynchronous-updates
* Contributors: Benjamin-Tan, Lemme, Michael Ferguson

0.2.17 (2016-08-23)
-------------------
* Merge pull request `#9 <https://github.com/mikeferguson/moveit_python/issues/9>`_ from mikeferguson/pyassimp_fix
  pyassimp is broken in 16.04, temporary work around so we can release
* Contributors: Michael Ferguson

0.2.16 (2015-11-02)
-------------------
* fix error message to have proper function name
* Contributors: Michael Ferguson

0.2.15 (2015-08-20)
-------------------
* Fix types in move_group_interface
* add planning_scene_diff as optional field
* Contributors: Michael Ferguson, Mikkel Rath Pedersen

0.2.14 (2015-05-22)
-------------------
* add no-wait behavior for move, pick, and place
* updates for compliance with PEP8
* Contributors: Aaron Blasdel, Michael Ferguson

0.2.13 (2015-04-18)
-------------------
* better handle removal of objects
* place has no attached_object_touch_links
* Contributors: Michael Ferguson

0.2.12 (2015-04-11)
-------------------
* fixup planner id
* Contributors: Michael Ferguson

0.2.11 (2015-04-08)
-------------------
* fix spelling issue in velocity scaling factor
* Contributors: Michael Ferguson

0.2.10 (2015-04-06)
-------------------
* implement kwargs for pick&place interface
* add max_velocity_scaling_factor as kwarg
* allow overriding allowed_planning_time
* add FakeGroupInterface
* add clear() method to planning scene interface
* Contributors: Michael Ferguson

0.2.9 (2015-03-28)
------------------
* add rotate_pose_msg_about_origin
* Contributors: Michael Ferguson

0.2.8 (2015-03-21)
------------------
* expose num_attempts through kw_args
* Contributors: Michael Ferguson

0.2.7 (2014-11-19)
------------------
* enable removing attached objects
* Contributors: Michael Ferguson

0.2.6 (2014-11-16)
------------------
* use kw_args for group interface, add a number of args supported
* a couple of fixes for is_diff
* Contributors: Michael Ferguson

0.2.5 (2014-07-09)
------------------
* add scripts for dumping/loading planning scene
* documentation cleanup
* Contributors: Michael Ferguson

0.2.4 (2014-06-03)
------------------
* add list and remove object scripts, closes `#2 <https://github.com/mikeferguson/moveit_python/issues/2>`_
* properly initialize planning scene interface, fixes `#1 <https://github.com/mikeferguson/moveit_python/issues/1>`_
* add planner_id logic to move group interface
* remove default support name
* fix comment on result type
* add missing import
* upstream the retry logic
* Contributors: Michael Ferguson

0.2.3 (2014-05-26)
------------------
* fix bug in mesh generation
* Contributors: Michael Ferguson

0.2.2 (2014-05-21)
------------------
* pass full result in pick and place
* Contributors: Michael Ferguson

0.2.1 (2014-05-19)
------------------
* fix pyassimp rosdep
* Contributors: Michael Ferguson

0.2.0 (2014-05-19)
------------------
* Initial release after forking from moveit_utils
* Contributors: Michael Ferguson
