# A packet of data for transfer via a network modem.

Header      header

string      source_ip   # The IP address of the data sender (auto-populated by driver)
uint8[]     data        # The data being transferred
