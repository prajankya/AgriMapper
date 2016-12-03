## The configuration is for wifi starting


Create a File here named **wpa_supplicant.conf** with this command

```bash
sudo wpa_passphrase NETWORK_NAME NETWORK_PASSWORD > wpa_supplicant.conf  
```

Then Update the connectWifi.bash file for ssid of wifi


then put the file connectWifi.bash in startup to switch on wifi automatically
