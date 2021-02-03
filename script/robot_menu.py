## Text menu in Python
import subprocess
import os

def print_menu():       ## Your menu design here
    print 30 * "-" , "ROBOT MENU" , 30 * "-"
    print "1. Serial scan"
    print "2. Test camera"
    print "3. cat lastboot.log "
    print "4. cat boot_init_rt2019.log"
    print "5. tim_init.sh"
    print "6. logp_restart.sh"
    print "7. cd /home/istrobotics/projects/istro_rt2019"
    print "8. Start vision_server"
    print "9. Start r.sh"
    print "0. Exit"
    print 72 * "-"
  
loop=True      
  
while loop:          ## While loop which will keep going until loop = False
    print_menu()    ## Displays menu
    choice = input("Enter your choice: ")
     
    if choice==1:     
        print "Serial scan"
        ## You can add your code or functions here
        os.system("python  ~/projects/python/serial_scan3.py")
        loop=False
    elif choice==2:
        print "Testing camera"
        ## You can add your code or functions here
        os.system("python ~/projects/python/OpenCV/camera_capture.py")
        loop=False        
    elif choice==3:
        print "cat lastboot.log:"
        ## You can add your code or functions here
        os.system("cat /home/istrobotics/bootscripts/lastboot.log")
        loop=False                
    elif choice==4:
        print "Menu 4 has been selected"
        ## You can add your code or functions here
        os.system("cat /home/istrobotics/projects/istro_rt2020/boot_init_rt2020.log")
        loop=False
    elif choice==5:
        print "Menu 5 has been selected"
        ## You can add your code or functions here
        os.system("/home/istrobotics/projects/istro_rt2020/script/tim_init.sh")
        loop=False           
    elif choice==6:
        print "Menu 6 has been selected"
        ## You can add your code or functions here
        cwd = os.getcwd()
        print cwd
        os.chdir('/home/istrobotics/projects/istro_rt2020/script')
        cwd2 = os.getcwd()
        print cwd2

        os.system("cd /home/istrobotics/projects/istro_rt2020/script/ && /home/istrobotics/projects/istro_rt2020/script/logp_restart.sh")
        os.chdir(cwd)
        cwd2 = os.getcwd()
        print cwd2
        

        loop=False
    elif choice==7:
        print "Menu 7 has been selected"
        ## You can add your code or functions here
        os.chdir('/home/istrobotics/projects/istro_rt2020')
        loop=False
    elif choice==8:
        print "Menu 8 has been selected"
        ## You can add your code or functions here
        os.chdir('/home/istrobotics/projects/istro_rt2020')
        os.system("cd /home/istrobotics/projects/istro_rt2020/visionn_server && /home/istrobotics/projects/istro_rt2020/visionn_server/visionn_server_start.sh")
        loop=False
    elif choice==9:
        print "Menu 9 has been selected"
        ## You can add your code or functions here
        os.chdir('/home/istrobotics/projects/istro_rt2020')
        os.system("cd /home/istrobotics/projects/istro_rt2020/ && ./r.sh")
        loop=False
    elif choice==0:
        print "Exit has been selected"
        ## You can add your code or functions here
        loop=False # This will make the while loop to end as not value of loop is set to False
    else:
        # Any integer inputs other than values 1-5 we print an error message
        raw_input("Wrong option selection. Enter any key to try again..")
