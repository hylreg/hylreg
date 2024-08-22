import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs

import ImageProvider 1.0

Window {
    height: 480
    title: qsTr("Hello World")
    visible: true
    width: 640

    Row {
        id: row

        height: 400
        width: 200

        Image {
            id: image

            fillMode: Image.PreserveAspectFit
            height: 200
            source: "qrc:/res/SSD.png"
            width: 200
        }
        Column {
            id: column

            height: 200
            width: 200

            Switch {
                id: _switch

                text: qsTr("打开摄像头")

                onCheckedChanged: {
                    if (_switch.checked) {
                        console.log("打开摄像头");
                        myImageProvider.openCamera(0);
                    } else {
                        console.log("关闭摄像头");
                    }
                }

                ImageProvider{
                    id: myImageProvider
                }
            }
            Row {
                id: row1

                height: 200
                width: 200

                TextField {
                    id: textField

                    placeholderText: qsTr("模型路径")
                }
                Button {
                    id: button

                    text: qsTr("选择模型")

                    onClicked: {
                        let filePath = fileDialog.open();
                    }

                    FileDialog {
                        id: fileDialog

                        nameFilters: ["All Files (*.*)", "Text Files (*.txt)", "Images (*.png *.jpg *.jpeg)"]
                        title: "Select a File"

                        onAccepted: {
                            textField.text = fileDialog.currentFile;
                            console.log(fileDialog.currentFile);
                        }
                        onRejected: {
                            console.log("File selection was canceled");
                        }
                    }
                }
            }
        }
    }
}
