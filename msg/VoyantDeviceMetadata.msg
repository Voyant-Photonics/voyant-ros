# VoyantDeviceMetadata.msg
std_msgs/Header header             # Standard ROS header with timestamp
string device_id                   # e.g., "CAR-30-005"
uint8 product_id                   # Device class behind device_id; 0 when the frame carries no state
uint32 serial_number               # Device serial behind device_id; 0 when the frame carries no state
string api_version                 # voyant-api library linked by this driver, e.g. "1.0.0"
string interface_contract_version  # sensor wire protocol the library was built for, e.g. "1.5.6"
string recording_api_version       # voyant-api that wrote the source recording; empty for a live capture
string firmware_version            # sensor MCU version; empty when the frame carries no state
string hdl_version                 # sensor FPGA version; empty when the frame carries no state
