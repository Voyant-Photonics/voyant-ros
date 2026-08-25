// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant_ros/mcap_to_bin.hpp"
#include "voyant_ros/conversion_utils.hpp"
#include <iomanip>
#include <iostream>
#include <map>
#include <voyant_data_recorder.hpp>
#include <yaml-cpp/yaml.h>

namespace voyant_ros
{

McapConfig load_config(const std::string &yaml_path)
{
  McapConfig config;

  try
  {
    YAML::Node yaml_config = YAML::LoadFile(yaml_path);
    config.mcap_input = yaml_config["mcap_input"].as<std::string>();
    config.bin_output = yaml_config["bin_output"].as<std::string>();
    config.topic_name = yaml_config["topic_name"].as<std::string>();
  }
  catch(const std::exception &e)
  {
    std::cerr << "Error parsing YAML: " << e.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }

  return config;
}

McapPlayback::McapPlayback(const McapConfig &config)
    : config_(config),
      validated_(false)
{
  storage_options_.uri = config_.mcap_input;
  storage_options_.storage_id = "mcap";

  converter_options_.input_serialization_format = "cdr";
  converter_options_.output_serialization_format = "cdr";
}

namespace
{

/// Whether `bag_topic` is the topic called `name`, allowing for a namespace: the driver
/// publishes relative names, so a capture under /front records /front/point_cloud. The
/// leading slash makes this a path-segment match -- /point_cloud_filtered is a different
/// topic, not a longer spelling of /point_cloud.
bool isTopic(const std::string &bag_topic, const std::string &name)
{
  const std::string suffix = name.empty() || name.front() == '/' ? name : "/" + name;
  return bag_topic.size() >= suffix.size() &&
         bag_topic.compare(bag_topic.size() - suffix.size(), suffix.size(), suffix) == 0;
}

/// The one topic in `topics` called `name`. Ambiguity is refused rather than guessed:
/// picking one of two sensors' streams would silently pair a cloud with the wrong
/// device's serial in the rebuilt recording.
bool resolveTopic(const std::vector<rosbag2_storage::TopicMetadata> &topics,
                  const std::string &name,
                  std::string &resolved)
{
  std::vector<std::string> matches;
  for(const auto &topic : topics)
  {
    // An exact hit settles it: voyant_bin_to_mcap writes topic_name verbatim, so a
    // relative name reaches the bag as-is, and a fully-qualified one must win over a
    // deeper namespace that merely ends the same way.
    if(topic.name == name)
    {
      resolved = topic.name;
      return true;
    }
    if(isTopic(topic.name, name))
    {
      matches.push_back(topic.name);
    }
  }

  if(matches.empty())
  {
    std::cerr << "✗ No '" << name << "' topic in this bag" << std::endl;
    return false;
  }
  if(matches.size() > 1)
  {
    std::cerr << "✗ Ambiguous '" << name << "': this bag has";
    for(const auto &match : matches)
    {
      std::cerr << " " << match;
    }
    std::cerr << "\n  Set topic_name to the full topic you want to convert." << std::endl;
    return false;
  }

  resolved = matches.front();
  return true;
}

} // namespace

void McapPlayback::openReader()
{
  reader_ = std::make_unique<rosbag2_cpp::Reader>();

  try
  {
    reader_->open(storage_options_, converter_options_);
  }
  catch(const std::exception &e)
  {
    std::cerr << "Failed to open MCAP file: " << e.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

bool McapPlayback::resolveTopics()
{
  const std::vector<rosbag2_storage::TopicMetadata> topics = reader_->get_all_topics_and_types();
  if(!resolveTopic(topics, config_.topic_name, cloud_topic_))
  {
    return false;
  }

  // The metadata must be the cloud's sibling, not merely the only one in the bag:
  // accepting /rear/device_metadata for /front/point_cloud would rebuild one sensor's
  // clouds under the other's identity.
  metadata_topic_ = deviceMetadataTopicFor(cloud_topic_);
  for(const auto &topic : topics)
  {
    if(topic.name == metadata_topic_)
    {
      return true;
    }
  }

  std::cerr << "✗ No '" << metadata_topic_ << "' topic in this bag, alongside " << cloud_topic_
            << std::endl;
  return false;
}

bool McapPlayback::validate()
{
  openReader();

  std::cout << "Validating file: " << config_.mcap_input << std::endl;

  // Resolved once, up front: both passes then compare topics exactly.
  if(!resolveTopics())
  {
    return false;
  }
  std::cout << "✓ Converting " << cloud_topic_ << " with " << metadata_topic_ << std::endl;

  bool metadata_found = false;
  bool first_frame_validated = false;

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serializer;
  rclcpp::Serialization<voyant_ros::msg::VoyantDeviceMetadata> metadata_serializer;

  // First pass - look for metadata and validate first frame
  while(reader_->has_next() && (!metadata_found || !first_frame_validated))
  {
    auto bag_message = reader_->read_next();

    // Check for metadata message
    if(bag_message->topic_name == metadata_topic_ && !metadata_found)
    {
      try
      {
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        metadata_serializer.deserialize_message(&serialized_msg, &metadata_);
        metadata_found = true;

        // api_version arrived with the v1.0.0 message, so an empty one means the writer
        // used an older definition -- either a pre-1.0.0 driver or a stale voyant_ros
        // install shadowing the one that wrote the bag.
        if(metadata_.api_version.empty())
        {
          std::cerr << "✗ Metadata has no api_version. Either the MCAP was recorded by a "
                       "pre-1.0.0 driver,\n  or it was written against a stale voyant_ros "
                       "install: check for an older\n  ros-$ROS_DISTRO-voyant-ros package "
                       "shadowing your build."
                    << std::endl;
          return false;
        }

        std::cout << "✓ Found metadata - Device ID: " << metadata_.device_id << " (recorded with "
                  << "voyant-api " << metadata_.api_version << ")" << std::endl;
      }
      catch(const std::exception &e)
      {
        // A pre-1.0.0 bag carries four version *hashes* where this build expects
        // strings, so it fails here rather than at the api_version check below.
        std::cerr << "✗ Failed to parse metadata: " << e.what()
                  << "\n  The bag's VoyantDeviceMetadata does not match this build: it was "
                     "written by a\n  pre-1.0.0 driver, or against a stale voyant_ros install."
                  << std::endl;
        return false;
      }
    }

    // Check first point cloud frame
    if(bag_message->topic_name == cloud_topic_ && !first_frame_validated)
    {
      try
      {
        sensor_msgs::msg::PointCloud2 cloud;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        pc_serializer.deserialize_message(&serialized_msg, &cloud);

        if(contains_valid_format(cloud))
        {
          first_frame_validated = true;
          std::cout << "✓ First frame contains a valid point format (" << cloud.width * cloud.height
                    << " points)" << std::endl;
        }
        else
        {
          std::cerr << "✗ First frame does not contain a valid point format" << std::endl;
          return false;
        }
      }
      catch(const std::exception &e)
      {
        std::cerr << "✗ Failed to parse first frame: " << e.what() << std::endl;
        return false;
      }
    }
  }

  if(!metadata_found)
  {
    std::cerr << "✗ No metadata message found" << std::endl;
    return false;
  }

  if(!first_frame_validated)
  {
    std::cerr << "✗ No valid point cloud frames found" << std::endl;
    return false;
  }

  // stateless() rejects an unrecognized ProductId, so probe it here rather than
  // failing at the first frame of the second pass, once the output file already exists.
  VoyantFrame::StatelessFrameDesc probe;
  probe.source = static_cast<ProductId>(metadata_.product_id);
  probe.serialNumber = metadata_.serial_number;
  if(!VoyantFrame::stateless(std::vector<PointData>{}, probe))
  {
    std::cerr << "✗ Metadata carries an unrecognized product_id ("
              << static_cast<int>(metadata_.product_id) << ")" << std::endl;
    return false;
  }

  std::cout << "✓ Validation complete" << std::endl;
  validated_ = true;

  // Close reader after validation
  reader_.reset();

  return true;
}

bool McapPlayback::processFrames()
{
  if(!validated_)
  {
    std::cerr << "Do not process frames without first validating" << std::endl;
    return false;
  }

  // Restart from beginning. Reopened before the recorder is constructed: openReader()
  // exits on failure, which would skip ~VoyantRecorder and orphan the output file.
  openReader();

  // Create recorder with (mostly) default configuration
  VoyantRecorderConfig recorder_config(config_.bin_output);
  recorder_config.timestampFilename = false; // Turn time-stamping the filename off
  VoyantRecorder recorder(recorder_config);

  if(!recorder.isValid())
  {
    // Construction opens the output file, so this covers a non-.vynt path and an
    // existing file as well as an unwritable one.
    std::cerr << "Failed to create VoyantRecorder: " << recorder.getLastError() << std::endl;
    return false;
  }

  std::cout << "\nProcessing frames..." << std::endl;

  size_t frame_count = 0;
  size_t recorded_count = 0;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serializer;

  // Second pass - process only point cloud frames
  while(reader_->has_next())
  {
    auto bag_message = reader_->read_next();

    // Process only point cloud messages
    if(bag_message->topic_name == cloud_topic_)
    {
      try
      {
        sensor_msgs::msg::PointCloud2 cloud;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        pc_serializer.deserialize_message(&serialized_msg, &cloud);

        // Convert PointCloud2 back to VoyantFrame and record
        try
        {
          VoyantFrame frame = convertPointCloud2ToFrame(cloud, metadata_);

          RecordResult result = recorder.recordFrame(frame);
          if(result == RecordResult::Error || result == RecordResult::Unknown)
          {
            std::cerr << "\nError recording frame " << frame_count << std::endl;
            return false;
          }
          if(result == RecordResult::Finished)
          {
            // A configured limit closed the log; this frame was not written.
            std::cerr << "\nRecorder stopped at frame " << frame_count << " (limit reached)"
                      << std::endl;
            break;
          }

          recorded_count++;
        }
        catch(const std::exception &e)
        {
          std::cerr << "\nFailed to convert frame " << frame_count
                    << " to VoyantFrame: " << e.what() << std::endl;
          return false;
        }

        frame_count++;
      }
      catch(const std::exception &e)
      {
        std::cerr << "Failed to parse frame " << frame_count << ": " << e.what() << std::endl;
        return false;
      }
    }
  }

  // Finalize recording
  std::cout << "\nFinalizing recording..." << std::endl;
  if(!recorder.finalize())
  {
    std::cerr << "Failed to finalize VoyantRecorder" << std::endl;
    return false;
  }

  std::cout << "\nProcessed " << frame_count << " frames" << std::endl;
  std::cout << "Successfully recorded " << recorded_count << " frames" << std::endl;
  std::cout << "Output saved to: " << config_.bin_output << std::endl;

  return true;
}

bool McapPlayback::contains_valid_format(const sensor_msgs::msg::PointCloud2 &cloud)
{
  // PCL maps a field only when name, datatype and count all match, and silently leaves
  // the destination zeroed otherwise -- so all three are checked here. Every field
  // below is a scalar, which PCL spells as count 1 or, tolerated, 0.
  using Field = sensor_msgs::msg::PointField;
  const std::map<std::string, uint8_t> required_fields = {{"x", Field::FLOAT32},
                                                          {"y", Field::FLOAT32},
                                                          {"z", Field::FLOAT32},
                                                          {"v", Field::FLOAT32},
                                                          {"snr", Field::FLOAT32},
                                                          {"drop_reason", Field::UINT8},
                                                          {"timestamp_nsecs", Field::UINT32},
                                                          {"azimuth_idx", Field::UINT16},
                                                          {"elevation_idx", Field::UINT16},
                                                          {"calibrated_reflectance", Field::FLOAT32},
                                                          {"frame_index", Field::UINT32}};

  std::map<std::string, const Field *> found_fields;
  for(const auto &field : cloud.fields)
  {
    found_fields.emplace(field.name, &field);
  }

  bool valid = true;
  for(const auto &[name, datatype] : required_fields)
  {
    auto found = found_fields.find(name);
    if(found == found_fields.end())
    {
      std::cout << "PointCloud missing required field: " << name << std::endl;
      valid = false;
    }
    else if(found->second->datatype != datatype)
    {
      std::cout << "PointCloud field '" << name << "' has type "
                << static_cast<int>(found->second->datatype) << ", expected "
                << static_cast<int>(datatype) << std::endl;
      valid = false;
    }
    else if(found->second->count > 1)
    {
      std::cout << "PointCloud field '" << name << "' has count " << found->second->count
                << ", expected a scalar" << std::endl;
      valid = false;
    }
  }

  return valid;
}

} // namespace voyant_ros
