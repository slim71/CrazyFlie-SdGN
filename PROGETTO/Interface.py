# -----------------------------------------------------------------------------
# --------------------------------------------------------------IMPORT---------
# -----------------------------------------------------------------------------
from random import *
import subprocess
from PySide2 import QtWidgets

# This is the import of the file where you wrote the code you want to execute.
# The name of the file has to be "interface_main_file" because it will be
# used also in other lines of this code.
import interface_main_file


# -----------------------------------------------------------------------------
# -----------------------------VARIABLES---------------------------------------
# -----------------------------------------------------------------------------
# This is the name of the file you want to show when you click on
# "instruction".
# In our case we call it "Readme.txt" but if you want to change it you just
# have to change the value of this string:
FILENAME = "README.txt"


# -----------------------------------------------------------------------------
# ----------------------------GRAPHICS-----------------------------------------
# -----------------------------------------------------------------------------

# This is the class we use to create all the graphic objects and structures.
class Dialog(QtWidgets.QDialog):
    NumGridRows = 3
    NumButtons = 4

    def __init__(self):
        super(Dialog, self).__init__()

        self.createMenu()
        self.createHorizontalGroupBox()
        # self.createGridGroupBox()
        # self.createFormGroupBox()

        self.bigEditor = QtWidgets.QTextEdit()
        self.bigEditor.setPlainText("Press Start for running..")

        # buttonBox = QtWidgets.QDialogButtonBox(
        # QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)

        # buttonBox.accepted.connect(self.accept)
        # buttonBox.rejected.connect(self.reject)

        mainLayout = QtWidgets.QVBoxLayout()
        mainLayout.setMenuBar(self.menuBar)
        mainLayout.addWidget(self.bigEditor)
        mainLayout.addWidget(self.horizontalGroupBox)
        # mainLayout.addWidget(self.gridGroupBox)
        # mainLayout.addWidget(self.formGroupBox)
        # mainLayout.addWidget(buttonBox)
        self.setLayout(mainLayout)

        self.setWindowTitle("Imperio")

    def createMenu(self):
        self.menuBar = QtWidgets.QMenuBar()

        self.fileMenu = QtWidgets.QMenu("&File", self)
        self.exitAction = self.fileMenu.addAction("E&xit")
        self.menuBar.addMenu(self.fileMenu)

        self.exitAction.triggered.connect(self.accept)

    def createHorizontalGroupBox(self):
        self.horizontalGroupBox = QtWidgets.QGroupBox("")
        layout = QtWidgets.QHBoxLayout()

        # This is the part where you can add or remove buttons, if you want.
        self.button = QtWidgets.QPushButton("START")
        self.button_instruction = QtWidgets.QPushButton("INSTRUCTION")

        layout.addWidget(self.button_instruction)
        layout.addWidget(self.button)

        # In this way we assign for each button the function we want to
        # execute at the "click event"
        self.button.clicked.connect(self.OpenClick)
        self.button_instruction.clicked.connect(self.OpenFile)

        self.horizontalGroupBox.setLayout(layout)

    def createGridGroupBox(self):
        self.gridGroupBox = QtWidgets.QGroupBox("Grid layout")
        layout = QtWidgets.QGridLayout()

        for i in range(Dialog.NumGridRows):
            label = QtWidgets.QLabel("Line %d:" % (i + 1))
            lineEdit = QtWidgets.QLineEdit()
            layout.addWidget(label, i + 1, 0)
            layout.addWidget(lineEdit, i + 1, 1)

        self.smallEditor = QtWidgets.QTextEdit()
        self.smallEditor.setPlainText("This widget takes up about two thirds "
                                      "of the grid layout.")

        layout.addWidget(self.smallEditor, 0, 2, 4, 1)

        layout.setColumnStretch(1, 10)
        layout.setColumnStretch(2, 20)
        self.gridGroupBox.setLayout(layout)

    def createFormGroupBox(self):
        self.formGroupBox = QtWidgets.QGroupBox("Form layout")
        layout = QtWidgets.QFormLayout()
        layout.addRow(QtWidgets.QLabel("Line 1:"), QtWidgets.QLineEdit())
        layout.addRow(QtWidgets.QLabel("Line 2, long text:"),
                      QtWidgets.QComboBox())
        layout.addRow(QtWidgets.QLabel("Line 3:"), QtWidgets.QSpinBox())
        self.formGroupBox.setLayout(layout)

    # These are the functions we define and that we want to execute at
    # the "click event"
    def OpenFile(self):
        # This function open the file that shows the instructions to follow
        # before execute the experiment
        subprocess.run(['open', FILENAME], check=True)

    # This is the function in which you have to insert the code you want to
    # execute.
    # In this example we :
    # - generate a random integer (just for have an example of how to call
    # tha main function with some parameter)
    # - in order to avoid consecutive clicks on the same button during the
    # execution of a "real code" like the experiment of the CrazyFlie,
    # we decide to assign to this button (after the first click) a function
    # that does nothing. In this way for multiple click it will be
    # like a single click.
    # - from the file "interface_main_file" we call the function "Imperio"
    # that is our example of "main function". In the reality you have to
    # change tha name "Imperio" with the name of the function from the
    # "interface_main_file" you want to run to start the experiment.
    # if you don't have a function like that, you have to change your
    # "interface_main_file" in order to create a function "Imperio" where you
    # insert all the code you want to run.
    # If there are also other functions in your main file this won't be a
    # problem: they will be imported with the "import interface_main_file",
    # but only "Imperio()" will be executed on the click.
    # In this example we pass the random integer we created at the first step.
    def OpenClick(self):
        number = randint(1, 10)
        self.button.clicked.disconnect(self.OpenClick)
        self.button.clicked.connect(self.Nothing)
        interface_main_file.Imperio(self, number)

    # In order to avoid consecutive clicks before the end of the program
    # launched at the first click, we decide to "disable" consecutive clicks
    # for the user.
    # This is done by replacing the assigned function to the button "start"
    # with this one:
    def Nothing(self):
        self.bigEditor.append(
            "\n You can't press Start more than once. If you want to launch "
            "a new execution, please close this interface and launch it "
            "again.")


# -----------------------------------------------------------------------------
# ---------------------------------MAIN----------------------------------------
# -----------------------------------------------------------------------------
# This is the code we run when we execute this file. It will open the
# interface form where you can click on two buttons:
# - instruction: it will show the " readme file "
# - start: it will execute the function " Imperio " or
#          the one you replaced it with
if __name__ == '__main__':
    import sys

    app = QtWidgets.QApplication(sys.argv)
    dialog = Dialog()
    sys.exit(dialog.exec_())
