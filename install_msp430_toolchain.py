################################################################################
# @brief SCRIPT TO INSTALL THE MSP430 TOOLCHAIN AND UTILS ON YOUR SYSTEM       # 
# @author: Carl Mattatall (cmattatall2@gmail.com)                              # 
#                                                                              # 
# @note MUST BE RUN WITH ROOT PERMISSIONS ON LINUX                             # 
#                                                                              # 
################################################################################
import pip
import sys
import platform
import os
import wget
import requests
import math
import errno
import zipfile


###############################################################################
## CONSTANTS. DON'T EVER TOUCH THESE
###############################################################################
 
toolchain_url = "http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSPGCC/9_2_0_0/export"

###############################################################################

# @brief abort if not python 3
def checkPythonVersion():
    if sys.version_info.major < 3: # python 3 must be the runtime
        raise Exception(os.path.basename(__file__) + " must be executed using Python 3")


# @brief detect if python runtime is executing in 32 bit or 64 bit environment
def systemIs32Bit():

    ## THERE HAS TO BE A BETTER WAY OF DOING THIS...
    if sys.maxsize > 2**32:
        return False
    else:
        return True


# @brief forcibly create symlink to @target at location @link_name
# @note overwrites existing symlink with name == @name
def symlink_force(target, link_name):
    try:
        os.symlink(target, link_name)
    except OSError as e:
        if e.errno == errno.EEXIST:
            os.remove(link_name)
            os.symlink(target, link_name)
        else:
            raise e

# @brief query wrapper to safely get y/n input from user
def query_yes_no(question, default="yes"):
    valid = {"yes": True, "y": True, "ye": True, "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = None

        # python 3 versus python 2 shenanigans
        if sys.version_info.major < 3:
            choice = raw_input().lower()
        else:
            choice = input().lower()

        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' (or 'y' or 'n').\n")


def install_linux_drivers():
    os.system("apt-get update -y")
    os.system("apt-get install -y libusb libusb-dev")
    os.system("apt-get install -y libreadline-dev") # for strange line endings
    os.system("udevadm control --reload-rules && udevadm trigger")
    current_workdir = os.getcwd()
    os.chdir("/opt/")
    os.system("rm -r msp_debug")
    os.system("mkdir msp_debug")
    os.chdir("msp_debug")
    os.system("pwd")
    os.system("rm -r mspdebug")
    os.system("git clone https://github.com/dlbeer/mspdebug.git")
    os.chdir("mspdebug")
    os.system("make && make install")
    os.chdir(current_workdir)

    # retrigger udev so LD linkages against changes udev rules.d and loads
    # libusb.so.x.x.x
    os.system("udevadm control --reload-rules && udevadm trigger")



def install_windows_drivers():
    print("BLAH CARL FORGOT TO IMPLEMENT THIS (OR HASN'T IMPLEMENTED IT YET)!!")
    exit(1)

def install_apple_drivers():
    print("BLAH CARL FORGOT TO IMPLEMENT THIS (OR HASN'T IMPLEMENTED IT YET)!!")
    exit(1)




# @brief install toolchain on windows
def install_windows():
    # IN ADDITION TO THE INSTALL SCRIPT, WE HAVE TO INSTALL THE USB DRIVERS 
    #https://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430_FET_Drivers/latest/index_FDS.html


    print("installing msp430 toolchain for " + platform.system())

    archive_ext = ".zip"
    if systemIs32Bit():
        toolchain_folder = "msp430-gcc-9.2.0.50_win32"
    else:
        toolchain_folder = "msp430-gcc-9.2.0.50_win64"
    
    download_url = toolchain_url + "/" + toolchain_folder + archive_ext
    #os.system("curl %s --output %s" % (download_url, toolchain_folder + archive_ext))

    toolchain_zip_name = str(toolchain_folder + archive_ext)
    toolchain_zipdata = requests.get(download_url)
    toolchain_zip = open(toolchain_zip_name, "wb")
    toolchain_zip.write(toolchain_zipdata.content)
    toolchain_zip.close()

    with zipfile.ZipFile(toolchain_folder + archive_ext, 'r') as zip_ref:
        zip_ref.extractall()
    os.system("rm \"%s\"" % (toolchain_folder + archive_ext))

    install_dir = "C:\\ti\\"
    os.mkdir("%s\\%s" % (install_dir, toolchain_folder))
    os.system("mv %s %s " % (toolchain_folder, install_dir))
    current_workdir = os.getcwd() # save to restore later
    os.chdir(install_dir + "\\" + toolchain_folder + "\\" + "bin")
    
    # NOW SHIT LIKE !!! THIS !!! is why programmers bitch and moan about windows being outdated and garbage
    # equivalent linux command is literally just PATH=$PATH;new/thing/to/add
    os.system("$PATH = [Environment]::GetEnvironmentVariable(\"PATH\")")
    os.system("$path_to_add = \"%s\"" % (os.getcwd()))
    os.system("[Environment]::SetEnvironmentVariable(\"PATH\", \"$PATH;$path_to_add\", \"Machine\")")
    os.chdir(current_workdir)
    os.system("echo $PATH")

    print("install_windows script is not finished yet!!")
    exit(1)

    install_windows_drivers()


# @brief install toolchain on linux
def install_linux():
    if os.geteuid() != 0:
        print(os.path.basename(__file__) + " must be executed with root permission")
        exit(1)
    else:
        print("installing msp430 toolchain for " + platform.system())

    archive_ext = ".tar.bz2"
    if systemIs32Bit():
        toolchain_folder = "msp430-gcc-9.2.0.50_linux32"
    else:
        toolchain_folder = "msp430-gcc-9.2.0.50_linux64"
    
    install_dir = "/opt"
    download_url = toolchain_url + "/" + toolchain_folder + archive_ext
    os.system("wget -qO- %s | tar -xj" % (download_url))
    os.system("rm -r %s/%s" % ( install_dir, toolchain_folder))
    os.system("mv %s %s " % (toolchain_folder, install_dir))
    current_workdir = os.getcwd() # save to restore later
    os.chdir(install_dir + "/" + toolchain_folder + "/bin")
    for executable in os.listdir(os.getcwd()):
        symlink_force(os.path.abspath(executable), "/usr/local/bin/" + executable)
    os.chdir(current_workdir)

    


    install_linux_drivers()




# @brief install toolchain on apple
# @todo ACTUALLY IMPLEMENT THE DAMN THING 
#      (I don't have a macbook so idk what to do yet)
def install_apple():
    print("installing msp430 toolchain for " + platform.system())
    toolchain_folder = "msp430-gcc-full-osx-installer-9.2.0.0.app.zip"
    download_url = toolchain_url + "/" + toolchain_folder

    print("install_apple script is not finished yet!!")
    exit(1)

    install_apple_drivers()


# @brief MAIN INSTALLATION WRAPPER
def install_toolchain():
    checkPythonVersion()
    if platform.system() == "Windows":
        install_windows()
    if platform.system() == "Linux":
        install_linux()
    elif platform.system() == "Apple":
        install_apple()        

if __name__ == "__main__":
    install_toolchain()