function usb_setup_arduino {
  port=$1

  # Check if USB is connected.
  ls /dev/tty"$1"0 > /dev/null 2>&1
  if [ ! $? -eq 0 ] 
  then
    echo "USB device not connected!"
    return
  fi

  # Get the information about the USB.
  idVendor=`( udevadm info -a -n /dev/tty"$1"0 | grep '{idVendor}' | head -n1 | xargs | grep -oP "==\K.*")`
  exit_status=$?
  idProduct=`( udevadm info -a -n /dev/tty"$1"0 | grep '{idProduct}' | head -n1 | xargs | grep -oP "==\K.*")` 
  (( exit_status = exit_status || $? ))
  serial=`( udevadm info -a -n /dev/tty"$1"0 | grep '{serial}' | head -n1 | xargs | grep -oP "==\K.*")`
  (( exit_status = exit_status || $? ))

  if [[ $exit_status -ne 0 ]]; then
    echo "Something went wrong while getting the info about the USB."
    return
  fi

  # Write the configuration to file.
  usb_conf="/etc/udev/rules.d/99-serial-arduino.rules"
  if [ -s "$usb_conf" ]
  then 
    echo "/etc/udev/rules.d/99-serial-arduino.rules already exists and is not empty! Set USB symlink manually."
  else
    echo "Config file does not exist, or is empty. Adding necessary lines."
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$idVendor\", ATTRS{idProduct}==\"$idProduct\", ATTRS{serial}==\"$serial\", SYMLINK+=\"tty_arduino\"" | sudo tee -a "$usb_conf" > /dev/null
  fi
}


function usb_setup_pozyx {
  port=$1

  # Check if USB is connected.
  ls /dev/tty"$1"0 > /dev/null 2>&1
  if [ ! $? -eq 0 ] 
  then
    echo "USB device not connected!"
    return
  fi

  # Get the information about the USB.
  idVendor=`( udevadm info -a -n /dev/tty"$1"0 | grep '{idVendor}' | head -n1 | xargs | grep -oP "==\K.*")`
  exit_status=$?
  idProduct=`( udevadm info -a -n /dev/tty"$1"0 | grep '{idProduct}' | head -n1 | xargs | grep -oP "==\K.*")` 
  (( exit_status = exit_status || $? ))
  serial=`( udevadm info -a -n /dev/tty"$1"0 | grep '{serial}' | head -n1 | xargs | grep -oP "==\K.*")`
  (( exit_status = exit_status || $? ))

  if [[ $exit_status -ne 0 ]]; then
    echo "Something went wrong while getting the info about the USB."
    return
  fi

  # Write the configuration to file.
  usb_conf="/etc/udev/rules.d/99-serial-pozyx.rules"
  if [ -s "$usb_conf" ]
  then 
    echo "/etc/udev/rules.d/99-serial-pozyx.rules already exists and is not empty! Set USB symlink manually."
  else
    echo "Config file does not exist, or is empty. Adding necessary lines."
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$idVendor\", ATTRS{idProduct}==\"$idProduct\", ATTRS{serial}==\"$serial\", SYMLINK+=\"tty_pozyx\"" | sudo tee -a "$usb_conf" > /dev/null
  fi
}
