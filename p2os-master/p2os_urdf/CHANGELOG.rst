^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package p2os_urdf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2017-08-01)
------------------
* Small xacro fix.
* Switch to format 2 (`#48 <https://github.com/allenh1/p2os/issues/48>`_)
  * Update p2os_launch to package.xml format 2.
  * Update p2os_driver to package.xml format 2.
  * Update p2os_urdf to package.xml format 2.
  * Update p2os_teleop package.xml to format 2.
  * Update p2os_msgs to package.xml format 2.
  * Update p2os_doc to package.xml format 2.
* Contributors: Hunter Allen, Hunter L. Allen

2.0.7 (2017-05-30)
------------------
* Changed to c++11 abi
* Updated changelog.
* merged.
* Updated package metadata, as well as added the correct c++ flags to the urdf file (for Gentoo support).
* Contributors: Hunter L. Allen

* merged.
* Updated package metadata, as well as added the correct c++ flags to the urdf file (for Gentoo support).
* Contributors: Hunter L. Allen

2.0.6 (2017-05-22)
------------------
* Change mass to more reasonable value.
* Clean up indentation.
* Clean up obsolete xml-schema namespaces.
* Remove obsolete files.
* Fix swivel joint.
* Add plugin for publishing ground truth odometry (position and velocity).
* Add and configure differential drive plugin.
  Seems to behave reasonably.
  There is an issue with the swivel which seems to break off randomly.
* Clean up and simplify collisions.
* Remove deprecated elem tag.
  Promote its attributes mu1, mu2, kp, kd to tags.
* Fix xacro deprecation warnings.
* Fix xacro deprecated warnings.
* Remove visual names in wheel descriptions...
  ...due to SDF bug https://bitbucket.org/osrf/sdformat/issues/132/parser-does-not-handle-urdf-material
* Use wheel descriptions from pioneer3dx_wheel.xacro.
  Remove invalid collision element from wheel definition.
  Comment out transmission element for now.
* Remove visual names.
  Visual names prevented Gazebo from rendering material colors, as per this issue: https://bitbucket.org/osrf/sdformat/issues/132/parser-does-not-handle-urdf-material
* Fix crash on model spawn in Gazebo.
  Remove zero-sized collision elements, as per this thread: http://answers.gazebosim.org/question/15816/gazebo-crashes-when-spawning-robot-from-urdf/
* Contributors: Damjan Miklic

2.0.5 (2016-05-26)
------------------
* Added missing dep.
* Contributors: Hunter L. Allen

2.0.4 (2016-05-26)
------------------

2.0.3 (2015-10-25)
------------------
* Updated p2os_urdf's build dependencies. Fixes `#39 <https://github.com/allenh1/p2os/issues/39>`_
* Updated the pioneer3at URDF xacro file. Fixes `#38 <https://github.com/allenh1/p2os/issues/38>`_
* Cleaned up the publisher file.
* Contributors: Hunter Allen

2.0.2 (2015-08-04)
------------------

2.0.1 (2015-07-11)
------------------
* Forgot a line.
* Working on Pioneer-3dx Gazebo.
* Contributors: Hunter Allen

1.0.13 (2015-05-02)
-------------------
* Pioneer-3dx circus bug is now fixed. Now to get it to move... We'll see.
* Updated the publisher to include pioneer-3dx frames.
* this fixes issue `#30 <https://github.com/allenh1/p2os/issues/30>`_
* removed gedit stuff
* fixed urdf pros publisher
* Removed unnecessary lib install.
  The line broke things and did nothing, so... Ya. It's gone now.
* Fixed launch
* Contributors: Guy Burroughes, Hunter Allen, Isura Ranatunga

1.0.12 (2014-06-25)
-------------------
* Updated to match indigo-devel
* General cleanup and fixing issues with the code
* Contributors: Aris Synodinos, Hunter Allen

1.0.11 (2014-06-25)
-------------------

1.0.10 (2014-05-28)
-------------------
* Added meshes directory to CMake install dir.
* Removed unneccessary files
* Contributors: Hunter Allen

1.0.9 (2013-08-18)
------------------
* Updated version
* 1.0.7
* Updated changelogs

1.0.7 (2013-08-18)
------------------

* Updated to match hmt-git.com repository

1.0.5 (2013-07-23)
------------------
* Cleaned up for release

* Updated to match hmt-git.com

* Updated to hmt-git.com repo

1.0.1 (2013-07-22)
------------------
* Updating to match hmt-git.com repo
* Updated the Pioneer-3dx to work with Gazebo in Hydro
* Changes to wheel collision geometry and Gazebo settings.
* Changes to wheel collision geometry (robot was unable to rotate).
* Added launch file
* Updated the Pioneer-3AT xacro to have colour in Gazebo, and added a launch file.
* URDF of 3AT divided into 2 files.
* Gazebo related changes.
* Small fix in xacro file.
* Renamed URDF executable
* Renamed URDF executable
* Renamed URDF executable
* Removed a line
* Removed xacro dependency
* removed xacro from package.xml
* Updated p2os_urdf to use catkin
* Added the URDF models for the Pioneer-3AT robots.
* Adding inertia for Gazebo simulation
* fixing
* fixing
* Fixing the urdf to work better
* Fixed the p2os_urdf.launch to work correctly
* Took credit for my work...
* cleaned up
* Updated the p2os_urdf package to work properly.
