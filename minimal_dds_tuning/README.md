# minimal_dds_tuning

## How to run

### rmw_fastrtps

TODO: customize profile for real-time applications

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=`ros2 pkg prefix minimal_dds_tuning`/share/minimal_dds_tuning/dds_profiles/rmw_fastrtps/DEFAULT_FASTRTPS_PROFILES.xml
```

### rmw_cyclonedds

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=`ros2 pkg prefix minimal_dds_tuning`/share/minimal_dds_tuning/dds_profiles/rmw_cyclonedds/cyclonedds.xml
```

### rmw_connextdds

TODO

## Resources

General
- https://docs.ros.org/en/galactic/How-To-Guides/DDS-tuning.html

rmw_fastrtps
- https://github.com/ros2/rmw_fastrtps
- ROS2 Fine Tunning ([slides](https://roscon.ros.org/2017/presentations/ROSCon%202017%20ROS2%20Fine%20Tuning.pdf))
- ROSCon 2019 Macau: ROS2 Real-Time Behavior: Static Memory Allocation ([video](https://vimeo.com/379127767))
- Allocations: https://fast-rtps.docs.eprosima.com/en/v1.9.0/realtime.html#tuning-allocations
https://fast-rtps.docs.eprosima.com/en/v1.9.0/realtime.html
- XML profiles configuration: https://fast-rtps.docs.eprosima.com/en/v1.9.0/xmlprofiles.html
- https://github.com/ros2/rmw_fastrtps#advance-usage
- Zero-copy support https://github.com/ros2/rmw_fastrtps/issues/579 https://github.com/ros2/rmw_fastrtps/issues/596

rmw_cyclonedds
- https://github.com/ros2/rmw_cyclonedds
- Thread configuration: https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/config.rst#thread-configuration
- Dynamic memory allocations: https://github.com/eclipse-cyclonedds/cyclonedds/issues/99
- Dynamic memory allocations: https://github.com/ros2/rmw_cyclonedds/issues/79

rmw_connextdds
- https://github.com/ros2/rmw_connextdds
- https://community.rti.com/static/documentation/connext-dds/5.2.0/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/Content/UsersManual/QoS_Profiles.htm#xmlconfiguration_1275484337_389138
- Thread settings: https://community.rti.com/static/documentation/connext-dds/5.2.0/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/ConfigThreadSettings_XML.htm#threading_3235324107_773338