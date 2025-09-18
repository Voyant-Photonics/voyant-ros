// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant_ros/mcap_to_bin.hpp"
#include <iostream>
#include <logging_utils.hpp>

int main(int argc, char *argv[])
{
  if(argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <input_yaml_file_path>\n";
    return EXIT_FAILURE;
  }

  // Initialize API internal logging
  voyant_log_init_c();

  // Load configuration
  auto config = voyant_ros::load_config(argv[1]);

  if(config.bin_output.empty())
  {
    std::cerr << "Error: bin_output path cannot be empty" << std::endl;
    return EXIT_FAILURE;
  }

  // Create playback manager
  voyant_ros::McapPlayback playback(config);

  // Phase 1: Validate metadata and first frame format
  if(!playback.validate())
  {
    std::cerr << "MCAP file validation failed" << std::endl;
    return EXIT_FAILURE;
  }

  // Phase 2: Process all frames (restarts from beginning)
  if(!playback.processFrames())
  {
    std::cerr << "MCAP frame processing failed" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "\nConversion complete!" << std::endl;
  return EXIT_SUCCESS;
}
