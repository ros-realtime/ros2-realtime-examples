# minimal_dds_tuning

## How to run

### rmw_fastrtps

TODO: customize profile for real-time applications

```bash
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=`ros2 pkg prefix minimal_dds_tuning`/share/minimal_dds_tuning/dds_profiles/rmw_fastrtps/REALTIME_FASTRTPS_PROFILES.xml
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
- [https://docs.ros.org/en/galactic/How-To-Guides/DDS-tuning.html](https://docs.ros.org/en/galactic/How-To-Guides/DDS-tuning.html)

rmw_fastrtps
- [https://github.com/ros2/rmw_fastrtps](https://github.com/ros2/rmw_fastrtps)
- ROSCon 2017 Vancouver: ROS2 Fine Tunning ([slides](https://roscon.ros.org/2017/presentations/ROSCon%202017%20ROS2%20Fine%20Tuning.pdf))
- ROSCon 2019 Macau: ROS2 Real-Time Behavior: Static Memory Allocation ([video](https://vimeo.com/379127767))
- [https://fast-rtps.docs.eprosima.com/en/v1.9.0/realtime.html#tuning-allocations](https://fast-rtps.docs.eprosima.com/en/v1.9.0/realtime.html#tuning-allocations)
- [https://fast-rtps.docs.eprosima.com/en/v1.9.0/realtime.html](https://fast-rtps.docs.eprosima.com/en/v1.9.0/realtime.html)
- [XML profiles configuration](https://fast-rtps.docs.eprosima.com/en/v1.9.0/xmlprofiles.html)
- [https://github.com/ros2/rmw_fastrtps#advance-usage](https://github.com/ros2/rmw_fastrtps#advance-usage)
- [How to enable LoanedMessages #579](https://github.com/ros2/rmw_fastrtps/issues/596)

rmw_cyclonedds
- [https://github.com/ros2/rmw_cyclonedds](https://github.com/ros2/rmw_cyclonedds)
- [Thread configuration](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/config.rst#thread-configuration)
- [About dynamic memory (de)allocation #99](https://github.com/eclipse-cyclonedds/cyclonedds/issues/99)
- [On allocations in the data path #79](https://github.com/ros2/rmw_cyclonedds/issues/79)

rmw_connextdds
- [https://github.com/ros2/rmw_connextdds](https://github.com/ros2/rmw_connextdds)
- [Configuring QoS with XML](https://community.rti.com/static/documentation/connext-dds/5.2.0/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/Content/UsersManual/QoS_Profiles.htm#xmlconfiguration_1275484337_389138)
- [Thread settings](https://community.rti.com/static/documentation/connext-dds/5.2.0/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/ConfigThreadSettings_XML.htm#threading_3235324107_773338)