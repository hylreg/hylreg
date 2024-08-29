import QtQuick
import QtQuick.Controls
import MyVision 1.0
import com.example.vision 1.0

Window {
    width: 1440
    height: 900
    visible: true
    title: qsTr("MyVision")

    Text {
        id: statusText
        text: Vision.status
        anchors.centerIn: parent
    }

    Button {
        text: "Update Status"
        anchors.bottom: statusText.bottom
        anchors.verticalCenterOffset: 40
        onClicked: {
            Vision.status = "Updated!"
        }
    }

    Button {
        text: "Get Status Info"
        anchors.bottom: statusText.bottom
        anchors.verticalCenterOffset: 80
        onClicked: {
            console.log(Vision.getStatusInfo())
        }
    }

    // MyComponent {
    //     id:myComponent
    // }


}
