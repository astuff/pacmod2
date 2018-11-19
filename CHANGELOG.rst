^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pacmod
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#3 <https://github.com/astuff/ros_pacmod/issues/3>`_ from astuff/maint/adding_roslint
  Maint/adding roslint
* Applying all recommended changes from roslint.
* Adding roslint as build requirement.
* Adding DBC file to repo.
* Changed 'overridden' to 'override_active' (name was changed in GlobalMsgRpt)
* Updating license in package.xml.
* Missing remaps for socketcan.
* Only sending commands when enabled.
* Listening for output instead of manual_input while disabled.
* When disabled, making command match report.
* Updating package.xml to format 2.
* Adding MIT license flower box to all source files.
* Documenting vehicle types before open-sourcing.
* Adding missing loop pause on can_write.
* Updating launch files for kvaser_interface.
* Changing topic names to match convention.
* Cleaning up launch file.
* Removing can_interface requirement. Minor clean-up.
* Merging in core pre-open-sourcing.
* Removing pause on CAN handle re-open.
* Fixing parsing bug in VinRpt.
* Fixing segfault with bad pointer.
* New encoding method.
* Making PacmodRosMsgHandler into PacmodTxRosMsgHandler.
* Changing read to new parsing method.
* Moving headers to pacmod_common.h.
* Adding VinRpt.
* Minor change to order of operations on connect.
* Making reader and writer connect to CAN the same way.
* Initializing CAN_IDs the right way.
* Moving all CAN_IDs to class properties - similar to other drivers.
* Cleaning up launch file formatting.
* Cleaning up package docs.
* Removing install of README which doesn't exist anymore.
* Keeping reader open.
* Adding a check for ROS while trying to connect.
* Add populate timestamps in messages
* Change steering pid 3 message, populate time stamp in rx echo, disable tx echo so tx_can doesn't include rx messages, add wheel speed and steer pid4 messages
* Fix scaling on steering PID messages
* Use sleep_until instead of sleep_for in can_send loop so it will send closer to the desired rate
* Adding license.
* Updating README.
* Fixing bug that causes thread to need killing if CAN channel is unavailable.
* More error handling. Now repeatedly attempts to connect.
  Added further error handling to canSend.
  Now attempts to connect both can_reader and can_writer with 1s delay
  time.
* Adding more error reporting. Shutting down on CAN open errors.
  Added more extensive error-reporting messages.
  On CAN open errors in either the reader or the writer, shut down (will
  be changed later).
* Matching changes made to can_interface.
* Fixing data types on DateTime message.
* Updating value types for LatLonHeading message.
* Added headlight and horn subscribers.
* Setting up more vehicle-specific publishing and subscribing. Updates to core.
* Only parsing headlights if vehicle is Lexus.
* Changes consistent with can_interface cleanup.
* Changing topic names to be more consistent with existing topics.
* Making some CAN publishing optional.
* Fixing missing yaw_rate_rpt_pub.
* Added parameter for vehicle_type. Added conditional publishers.
  Also added needed messages which include timestamps. Completed conditional logic
  for POLARIS_GEM, POLARIS_RANGER, INTERNATIONAL_PROSTAR_122, and LEXUS_RX_450H.
* Adding parsing support for messages from additional supported vehicles.
* Added code to handle semi windshield wipers
* Making canSend more efficient.
* Reset all values to default when PACMod is disabled.
* Parsing error messages on general report.
* Fixed scale value (10->1000) in SystemRptFloatMsg
* Setting clear_override to always be true.
* Changing to periodic message burst instead of ad-hoc transmission.
* Minor code cleanup
* Changing can_rx_forward to can_rx.
* Migrating from can_interface/CanFrame to can_msgs/Frame.
* added new topic as_tx/vehicle_speed for m/s speed
* changed how vehicle speed is calculated
* Fixes for heartbeat and vehicle speed.
* Removing initial enable/disable.
* Adding override debounce.
* Sends heartbeat. Listens for override on PACMod and adjusts heartbeat signal accordingly.
* Creating separate messages package. Cannot remove C++11 requirement - need mutexes.
* Contributors: Christopher Vigna, Daniel Stanek, Joe Driscoll, Joe Kale, Joshua Whitley, Sam Rustan, Nathan Imig
