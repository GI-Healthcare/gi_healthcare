#!/usr/bin/env python

'''
0 - firstScreen
1 - secondScreen
2 - inventoryScreen
3 - allergyScreen
4 - chatGPTScreen
5 - availableDishScreen
6 - noodleScreen
7 - vegetableSlot
8 - cookingScreen
9 - shutdownScreen
'''

import sys
import time
import os

#import pyttsx3

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QDialog, QApplication
from PyQt5.uic import loadUi
import images_qrc

import rospy
from std_msgs.msg import String

user = os.getlogin()

class MainWindow(QDialog):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.currentDir = "/home/"+user+"/catkin_ws/src/gi_healthcare/ui"
        ui_dir = self.currentDir + "/userInterface.ui"
        loadUi(ui_dir, self)

        self.screens.setCurrentIndex(0)

        self.status = "start"

        # Variables
        self.chatGPTResponse = ""
        self.selectedVegetable = list()
        self.selectedMeat = list()
        self.selectedDairy = list()
        self.selectedAllergy = list()
        self.cookingTime = 0.01      # cookingTime * 100 total time
        self.washingTime = 0.1      # washingTime * 100 total time

        # Chopping modes
        self.choppingMode.addItem("Select Chopping Mode")
        self.choppingMode.addItem("Big Slice")
        self.choppingMode.addItem("Small Slice")
        self.choppingMode.addItem("Battonet")
        self.choppingMode.addItem("Julienne")
        self.choppingMode.addItem("Large Dice")
        self.choppingMode.addItem("Medium Dice")
        self.choppingMode.addItem("Small Dice")

        self.washLogo.setHidden(True)
        self.cookLogo.setHidden(False)

        # ------------------- screen 1 ------------------- #
        self.startButton.clicked.connect(self.gotoSecondScreen)
        self.shutdownButton.clicked.connect(self.shutdownProcess)

        # ------------------- screen 2 ------------------- #
        self.customDishButton.clicked.connect(self.gotoInventoryScreen)
        self.availableCuisinesButton.clicked.connect(self.gotoAvailableScreen)
        self.back2_1Button.clicked.connect(self.gobackFirstScreen)

        # ------------------- screen 3 ------------------- #
        self.continueCustomButton.setEnabled(False)
        self.continueCustomButton.clicked.connect(self.gotoAllergyScreen)
        self.back31_2Button.clicked.connect(self.gobackSecondScreen)

        self.vegetables = [self.onionButton, self.potatoButton, self.garlicButton, self.tomatoButton]
        self.meat = [self.chickenButton, self.fishButton, self.porkButton, self.turkeyButton]
        self.dairy = [self.milkButton, self.eggButton, self.butterButton, self.creamButton]

        # vegetables event callback
        self.vegetables[0].clicked.connect(lambda: self.selectVegetable(self.vegetables[0], "onion"))
        self.vegetables[1].clicked.connect(lambda: self.selectVegetable(self.vegetables[1], "potato"))
        self.vegetables[2].clicked.connect(lambda: self.selectVegetable(self.vegetables[2], "garlic"))
        self.vegetables[3].clicked.connect(lambda: self.selectVegetable(self.vegetables[3], "tomato"))

        # meat event callback
        self.meat[0].clicked.connect(lambda: self.selectMeat(self.meat[0], "chicken"))
        self.meat[1].clicked.connect(lambda: self.selectMeat(self.meat[1], "fish"))
        self.meat[2].clicked.connect(lambda: self.selectMeat(self.meat[2], "pork"))
        self.meat[3].clicked.connect(lambda: self.selectMeat(self.meat[3], "turkey"))

        # dairy event callback
        self.dairy[0].clicked.connect(lambda: self.selectDairy(self.dairy[0], "milk"))
        self.dairy[1].clicked.connect(lambda: self.selectDairy(self.dairy[1], "egg"))
        self.dairy[2].clicked.connect(lambda: self.selectDairy(self.dairy[2], "butter"))
        self.dairy[3].clicked.connect(lambda: self.selectDairy(self.dairy[3], "cream"))

        # ------------------- screen 4 ------------------- #
        self.continueAllergyButton.setEnabled(False)
        self.continueAllergyButton.clicked.connect(self.gotoChatGPTScreen)
        self.back31_4Button.clicked.connect(self.gobackInventoryScreen)

        self.allergy = [self.peanuts, self.milk, self.seaFood, self.wheat, self.eggs, self.noAllergy]

        # allergy event callback
        self.allergy[0].toggled.connect(lambda: self.selectAllergy(self.allergy[0], "peanuts allergy"))
        self.allergy[1].toggled.connect(lambda: self.selectAllergy(self.allergy[1], "milk allergy"))
        self.allergy[2].toggled.connect(lambda: self.selectAllergy(self.allergy[2], "sea food allergy"))
        self.allergy[3].toggled.connect(lambda: self.selectAllergy(self.allergy[3], "wheat allergy"))
        self.allergy[4].toggled.connect(lambda: self.selectAllergy(self.allergy[4], "eggs allergy"))
        self.allergy[5].toggled.connect(lambda: self.selectAllergy(self.allergy[5], "no allergy"))

        # ------------------- screen 5 ------------------- #
        self.startCustomButton.clicked.connect(self.gotoCookingScreen)
        self.back4_5Button.clicked.connect(self.gobackAllergyScreen)

        # ------------------- screen 6 ------------------- #
        self.back32_2Button.clicked.connect(self.gobackSecondScreen)
        self.noodleButton.clicked.connect(self.gotoNoodleScreen)

        # ------------------- screen 7 ------------------- #
        self.startNoodlesCookButton.clicked.connect(self.gotoVegeSlotScreen)
        self.back1_7Button.clicked.connect(self.gobackAvailableScreen)

        # ------------------- screen 8 ------------------- #
        self.vegeSlotContiueButton.clicked.connect(self.gotoCookingScreen)
        self.back32_7Button.clicked.connect(self.gobackNoodlesScreen)

        # ------------------- screen 9 ------------------- #
        self.startWashButton.clicked.connect(self.gobackFirstScreenWash)

        # ------------------- screen 10 ------------------- #
        self.shutdownYes.clicked.connect(self.closeApplication)
        self.shutdownNo.clicked.connect(self.gobackFirstScreen)

        self.show()

    # ------------------- screen 1 ------------------- #
    def gotoSecondScreen(self):
        self.screens.setCurrentIndex(1)
        self.status = "start"
        pub.publish(self.status)
        print()

    def shutdownProcess(self):
        self.screens.setCurrentIndex(9)

    # ------------------- screen 2 ------------------- #
    def gotoInventoryScreen(self):
        self.screens.setCurrentIndex(2)

    def gotoAvailableScreen(self):
        self.screens.setCurrentIndex(5)

    def gobackFirstScreen(self):
        self.screens.setCurrentIndex(0)

    # ------------------- screen 3 ------------------- #
    def gotoAllergyScreen(self):
        self.screens.setCurrentIndex(3)

    def gobackSecondScreen(self):
        self.screens.setCurrentIndex(1)

    def selectVegetable(self, button, vegetable):
        self.continueCustomButton.setEnabled(True)
        if button.isChecked():
            self.selectedVegetable.append(vegetable)
        else:
            self.selectedVegetable.remove(vegetable)

    def selectMeat(self, button, meat):
        self.continueCustomButton.setEnabled(True)
        if button.isChecked():
            self.selectedMeat.append(meat)
        else:
            self.selectedMeat.remove(meat)

    def selectDairy(self, button, dairy):
        self.continueCustomButton.setEnabled(True)
        if button.isChecked():
            self.selectedDairy.append(dairy)
        else:
            self.selectedDairy.remove(dairy)

    # ------------------- screen 4 ------------------- #
    def gotoChatGPTScreen(self):
        self.screens.setCurrentIndex(4)
        self.selectedVegetable = list(dict.fromkeys(self.selectedVegetable))
        self.selectedDairy = list(dict.fromkeys(self.selectedDairy))
        self.selectedMeat = list(dict.fromkeys(self.selectedMeat))
        self.selectedAllergy = list(dict.fromkeys(self.selectedAllergy))
        vegetable = ','.join(self.selectedVegetable)
        meat = ','.join(self.selectedMeat)
        dairy = ','.join(self.selectedDairy)
        alergy = ','.join(self.selectedAllergy)
        chatGPTQuery = "Make a recipe only using " + vegetable + "," + meat + "," + dairy + " for a person with " + alergy
        self.chatGPTRes(chatGPTQuery)
        self.chatGPTSuggestion.setPlainText(self.chatGPTResponse)

    def chatGPTRes(self, query):
        # session_token = 'eyJhbGciOiJkaXIiLCJlbmMiOiJBMjU2R0NNIn0..3BVF8qeaLkmR4asw.XYmWXI1U4RflEakX6x3N-BaNtQJ6yZLCEC_dr4Oj32ZGevBsCX1Se8AYQ1NEyHe_m1pyPCOOV4pa--bFwwUdnVnCijzXQ3tXYZ4NsI9laogF7DQ44LxVnD31TwKdYLwxeRYVWaradp_wwILcPp-XrSF2h6nKQiqI_inW-9Sdit3oeOMQaikIIZgLbIm0kloyMVGwU38GUiUB3FPB-Awi4eBybuLiCkt06-VGXwcmfsRsqbEk2LwTv0AxwlWX6BGlDD1IlRzdiH7mKKBy_G1AyePtWhC4NrQDNtHuWHbvFmaRLogGma4iFQpU49Cq_9k3zFvfBhtl5IRZo8vhYaJ3IBZijw0fC_53kQUUjn7U_ha0ffmTYdKlTIpCcbEe25eUjeZj66S1NibORr5Noyzz6Or1SVfzUKz3S-lqNpPIhRyin4HXU0TUqCzvO5X8ear9WDMvF8QmkQHt9aTLeyaGcc_uqQO0fgFIy4e_V1TDKxoUOAoEUmY1fu7ZwLR-5-Tdf5XkkmCkmduQdQuvF9toCf9zKZe3yR3FFfzn982UAlzsxoQabC8SSrXAmVIjGFkYzI6RHLfZpj1SaIst7tylyX9INRGvq10quueKZU0QR7XVPzW7PxSPgD2E_mQm6zd8vhWRicoKlBhRdUwAWI_j6b3XYLn3km684_Mu1UL793chhxH5jNBxzX2VZ0rfZM30mjK2coX3ZFu0bJthavtHMeLMJaLWPNpGG_uOS8Cn6vE5nw8wNy6IyT5Gupnl44Aa78K-Mu5dR984MAApEZVYJ-mILWGMDfVy-eIICqtK0ENdzAvO4odf5P3sZgTSALccChfIFwJHExp96opm9ArQaXHWHL6QFL-j9cpgTAhMDIcrTLZ6foroQkC2SmqbuYId3sx-QwTH5WW6Qmn6MxLGD_bNTPUsbkSEAkRFidTuskowHPeyV23CXVLIrd7P-4hAiORtp73IXA3UI7eOoOft2l_zXLUKNO015UTr7In0cl715exQ5OHJfF7NFKit5ELGWBfYQTlQuhT0Pr9dkaP_FLm_lVzk1KTfseI8o6g6c8KExGm9U4or_FG5hH2-yZovjw2ntwR7sKVvH3Cz6doB2MsVs6RI9loa34-r99lM9eBSji6cCdBbHde8gkWknnnl2xkp1bnHfJR89wzdVlBPaIn-JEjpff-qcMFcc9_D7gP69QLlifBl0VyS5wLgL082nYxff8KaOmTUw2hd-_yWFjERrMr9MFgqCbRmS2Rso8RHU6hCxMmAipmbZd_FDjFsvAN9hyS6uAALnQgDw9YbgwKYueaI2gSrayKYPlUbuWycCrRlzErcqt6Ls4OcoD_dqOS8w8T_gPAv_emHkIIf69MNpg74awkNINCUh7IOSOCltCjmhF0mFFDDCXQc57XHBlIFYdubdu7BBNUHa1g85eNV4Yr1VvkU451h7dOnkhPIiYy_B7yFxV4KQ_GGjX2lf4hFx-vMqdUmEWdyXsLx0rhj9OArA6JjHeo6aqdjWVhmmxL0qHYPPzveIX8fwwJ9RwvdwhClPzKbFfml5Ob4b4eOW2IEOdujbRBPkyM8BzCEPRddA99IWdextZtkPR4dxk-KtzckAi0e5rP8b1pE7tYwhUJfW9bOdTdS38p2fV9ZDjkfqHMAFbAtI2pZWFXUfBv7pFbO45ayNbr-MiSo2QkQ93scbqtuVufusMpxg6tD0DD1oi1y0C0sMrhZsH9gRaWPmFcAJsr1iDwD9y6BRCjKDZOSxbxxh9NmmiPwHmPbA_hFo9JOfg8p4C5PfDE-igf3xobRW5VuEEmJi2OGIS93pacyNdGAgzx_-GzXbzlPFNk_rMKaLi-0n9bLfsh93lvV5t68Br8FZbHjkF9h4G2cvcgGA7UxMGfeYtg36ggYOTIQ2BZXBVm9dA1B-BKN-v3w-FpEKPPrf9X-YMP4uBrPfS3rtXF93sRQVNvXs8Luvak1WJQcaIYjLOW-Tkjc3zSB9UIE0uW9-iAHHAoUFxvD7bq2aHS6QGNjTNZKhkGpCh7WoDdc6Em-hVDF4gXhnCwOYA1wKyruBA78qlTwbZ9mbGaoJK3UhCA7LkNSouPkX43RVvNH07BkMVPlkKSO409tfRRURJLkvKZ0wH4W9-mMGundredvpY3WcQpBPsP7lnrAM6GkTVf6K7iihp0g8vXnZLi0qYugqgnLXe135_Usy8A6sPTogVDC0-KEufSgynVXImzZYtDc7hoAQc8KQigVDtHI6WadoI1qz__XUQghXErjFK6sWBDujxYspyw9ZhZtuVbXUDHP40a1EiJjnVOO_vTMb_sfjHd2VpHLmzoYJroX6K48GtqPP8x31LUUdDWKMI2ketI._zEtZzmrYxlpGVDRCDRyVg'  # `__Secure-next-auth.session-token` cookie from https://chat.openai.com/chat
        # api = ChatGPT(session_token)
        # resp = api.send_message(query)
        # self.chatGPTResponse = resp['message']
        # api.reset_conversation()
        # api.clear_conversations()
        # api.refresh_chat_page()
        pass

    def gobackInventoryScreen(self):
        self.screens.setCurrentIndex(2)

    def selectAllergy(self, check, alg):
        allergy = [self.peanuts, self.milk, self.seaFood, self.wheat, self.eggs, self.noAllergy]
        if (check.isChecked() == True) and (alg == "no allergy"):
            self.continueAllergyButton.setEnabled(True)
            self.selectedAllergy.append(alg)
            for i in range(0, len(allergy)-1):
                allergy[i].setCheckable(False)
        elif check.isChecked() == True:
            self.continueAllergyButton.setEnabled(True)
            allergy[len(allergy)-1].setCheckable(False)
            self.selectedAllergy.append(alg)

        if (check.isChecked() == False) and (alg == "no allergy"):
            self.selectedAllergy.remove(alg)
            for i in range(0, len(allergy)-1):
                allergy[i].setCheckable(True)
        elif check.isChecked() == False:
            self.selectedAllergy.remove(alg)
            if len(self.selectedAllergy) == 0:
                allergy[len(allergy)-1].setCheckable(True)

    # ------------------- screen 5 ------------------- #
    def gotoCookingScreen(self):
        self.status = "cooking"
        pub.publish(self.status)
        self.startWashButton.setEnabled(False)
        speech("Cooking started")
        self.screens.setCurrentIndex(8)
        self.cookingStatus.setText("Cooking in Progress")
        speech("Cooking in progress")
        self.completed = 0
        while self.completed < 100:
            self.completed += 1
            self.cookingProgress.setValue(self.completed)
            time.sleep(self.cookingTime)
            if self.completed == 99:
                self.cookingStatus.setText("Completed")
        self.startWashButton.setEnabled(True)
        speech("Cooking completed. Please take the serving bowl from the tray.")
        speech("Would you like to start the washing procedure?")

    def gobackAllergyScreen(self):
        self.screens.setCurrentIndex(3)

    # ------------------- screen 6 ------------------- #
    def gotoNoodleScreen(self):
        self.screens.setCurrentIndex(6)

    # ------------------- screen 7 ------------------- #
    def gobackAvailableScreen(self):
        self.screens.setCurrentIndex(5)

    def gotoVegeSlotScreen(self):
        self.screens.setCurrentIndex(7)

    # ------------------- screen 8 ------------------- #
    def gobackNoodlesScreen(self):
        self.screens.setCurrentIndex(6)

    # ------------------- screen 9 ------------------- #
    def gobackFirstScreenWash(self):
        self.cookLogo.setHidden(True)
        self.washLogo.setHidden(False)
        self.status = "washing"
        pub.publish(self.status)
        self.cookingStatus.setText("Washing in progress")
        speech("Washing in progress.")
        self.completed = 0
        while self.completed < 100:
            self.completed += 1
            self.cookingProgress.setValue(self.completed)
            time.sleep(self.washingTime)
        speech("Washing procedure completed. Thank you.")
        self.screens.setCurrentIndex(0)
        self.cookLogo.setHidden(False)
        self.washLogo.setHidden(True)

    # ------------------- screen 10 ------------------- #
    def closeApplication(self):
        self.status = "shutdown"
        pub.publish(self.status)
        time.sleep(2)
        app.quit()
    

# speech output
def speech(command, flag=False):
    if flag:
        #eng = pyttsx3.init() #initialize an instance
        #voice = eng.getProperty('voices') #get the available voices
        #eng.setProperty('rate', 180)
        #eng.setProperty('voice', voice[1].id) #changing voice to index 1 for female voice
        #eng.say(command) #say method for passing text to be spoken
        #eng.runAndWait()
        pass
    else:
        pass
        
# main
if __name__ == "__main__":
    rospy.init_node("User_Interface")

    pub = rospy.Publisher("ui_cmd", String, queue_size=1)

    app = QApplication(sys.argv)
    widget = QtWidgets.QStackedWidget()
    mainwindow = MainWindow()
    widget.addWidget(mainwindow)
    widget.setFixedWidth(480)
    widget.setFixedHeight(760)
    widget.show()
    sys.exit(app.exec_())