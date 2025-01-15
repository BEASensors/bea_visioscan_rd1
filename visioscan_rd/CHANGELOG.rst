1.2.5 (2025-1-2)
----------------
* Fix angle range mapping


1.2.4 (2024-12-19)
----------------
* Use skipspots to calculate angle increment
* Format style


1.2.3 (2024-09-23)
----------------
* Enable AR 0.05° and 0.025°


1.2.2 (2024-03-28)
----------------
* Restore single thread to handle received data and publish scandata
* Use a timer thread to monitor the connection and conduct reconnection operation
* Replace "VISIOSCANRD" with "VISIOSCAN"


1.2.1 (2024-03-20)
----------------
* Delete command interface socket before creating socket for tcp capturing
* Change timer frequency to 4 times scanner frequency
* Rename "E449" to "VISIOSCANRD"


1.2.0 (2024-03-19)
----------------
* Refactor the driver to add reconnection mechanism
* Use different threads to store the received data and publish the scandata
* Stop mdi data transferring before retrieving parameters


1.1.5 (2024-02-22)
----------------
* Bugfix startup failure of TCP mdi channel


1.1.4 (2024-02-08)
----------------
* Fix MDI transmission in TCP


1.1.3 (2023-08-04)
----------------
* Bugfix scan_time and time_increment in sensor message


1.1.2 (2023-07-07)
----------------
* Bugfix decoding intensity value
* Change paramerter of data output direction to sensor mounting direction

1.1.1 (2023-06-27)
----------------
* Set topic and frame name as parameter
* Set data output direction as parameter
* Bugfix pulse width value
* Change project and package name

1.1.0 (2023-06-09)
----------------
* Adapt to new MDI format v2

1.0.0 (2021-11)
----------------
* Initial release
* Contributors: ShaYaoyao@srobots
