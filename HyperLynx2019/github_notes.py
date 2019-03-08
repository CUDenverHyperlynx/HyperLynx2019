"""
How to get sensor libraries and such for you Raspbery Pi

    Check your installed python modules
    Open the terminal and enter the following commands hitting enter after each:
    
        python
        help()
        modules
        
    This will bring up a list of the python modules you have installed
    You are looking for
    
        pip
        git
        smbus
    
    If it pulls up an error when you enter modules, you can try
    
        modules pip
        modules git
    
    or from the terminal
    
        git --version
        pip --version
        
    to verify you have them.
    
    To get the sensor libraries, from the terminal:
    
        sudo pip install "moduleName"
    
    This is supposed to work for packages found at https://pypi.org, like
       
        Adafruit_BNO055
        Adafruit_BME280 
    
    But I had trouble getting all the libraries this way.       
    Any module you cannot install this way should be on our github repository

    To clone from github, first enter the desired destination in the terminal.
    
    ex:
        cd /home/pi/HyperLoop
    
    from there enter
        
        git clone "github link"
    ex:
        git clone https://github.com/CUDenverHyperlynx/Sensor_libraries
        
    This would get you everything from our Sensor_libraries repository on your pi.
    
        git clone https://github.com/CUDenverHyperlynx/Hyperlynx2019
    
    This would get you everything from our Hyperlynx2019 repository.
"""


