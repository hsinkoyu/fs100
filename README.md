# YASKAWA FS100 High Speed Ethernet Server Functions
  This class implements most of YASKAWA FS100 High Speed Ethernet Server Functions.  
  https://www.motoman.com/getmedia/16B5CD92-BD0B-4DE0-9DC9-B71D0B6FE264/160766-1CD.pdf.aspx?ext=.pdf

  Attributes:  
      ip (str): IP address of the controller  
      timeout (int, optional): Communication timeout value in second between PC and Controller. Defaults to 0.8.  
      errno (int): Number of last error  

  Methods:  
      switch_power(): Turn on/off the power supply  
      move(): Make robot move to one or more specified position(s)  
      stop(): Stop moving robot  
      one_move(): Make Robot move to the specified position  
      select_cycle(): Select the way a job in pendant plays  
      select_job(): Select a job in pendant for later playing  
      play_job(): Start playing a job in pendant  
      read_executing_job_info(): Read the info of executing job  
      read_axis_name(): Read the name of each axis  
      read_position(): Read the robot position  
      read_torque(): Read the robot torque data of each axis  
      read_variable(): Read a robot variable  
      write_variable(): Write a robot variable  
      get_status(): Retrieve various status of the robot  
      read_alarm_info(): Retrieve info of the specified alarm  
      get_last_alarm(): Retrieve info of the latest alarm  
      reset_alarm(): To reset alarms or cancel errors  
      acquire_system_info(): Acquire system information  
      acquire_management_time(): Acquire usage time of an action  
      show_text_on_pendant(): Show text on pendant  
      get_file_list(): Retrieve list of files ended with `extension` in pendant  
      send_file(): Send a local file to pendant  
      recv_file(): Receive a file from pendant  
      delete_file(): Delete a file in pendant  

# Simple GUI
![Alt text](doc/screenshot1.png?raw=true "Title")

![Alt text](doc/screenshot2.png?raw=true "Title")

![Alt text](doc/screenshot3.png?raw=true "Title")
