### Dynamic UI rqt plugin

Have you ever wanted to create a small UI wihtout any business logic for publishing or visualizing some data from your ROS system?

Now you can create this without any coding using the rqt_dynamic_ui plugin!

## Usage workflow:

 - Design your user interface with QtDesigner or the Designer component of the QtCreator
 - Add topic pubish/subscription properties to your widgets
 - Load the saved ui file to rqt with the rqt_dynamic_ui plugin
 - Enjoy your user interface!

## TODO 

 - Implement all features

## Supported widgets:

### QSpinBox/QDoubleSpinBox

Publishing properties:
- publishTopic: the topic/topic data field to publish data
- publishDataType: one data type from the supported numeric std_msgs types. If this property is not present std_msgs::Int32 and Float32 is going to be used (for QSpinBox/QDoubleSpinBox respectively).
- publishEvent: possible values: editingFinished/valueChanged. If the widget does not have this property then the data will be published on the valueChanged signal (once the value is changed).

Subscribing properties **(not yet implemented)**:
- subscribeTopic: the widget will display the value published to this topic. It is useful to set the readOnly property of the widget if this feature is used.

### QDial/QSlider

Publishing properties:
- publishTopic: the topic/topic data field to publish data
- publishDataType: one data type from the supported numeric std_msgs types. If this property is not present std_msgs::Int32 going to be used.
- publishEvent: possible values: sliderReleased/valueChanged. If the widget does not have this property then the data will be published on the valueChanged signal (as the slider got moved).

Subscribing properties **(not yet implemented)**:
- subscribeTopic: the widget will display the value published to this topic. It is useful to set the readOnly property of the widget if this feature is used.

### QPushButton/QToolButton/QCheckBox/QRadioButton

Publishing properties:
- publishTopic: the topic/topic data field to publish data upon the button interaction specified in the publishEvent
- publishDataType: one data type from the supported numeric std_msgs types. If this property is not present std_msgs::Int32 going to be used.
- publishCheckstate: if the widget has this property then a 0 or 1 (depending on the checked state) will be published to the publishTopic once the button got checked/unchecked. 
- publishDataOnClick: the data to be published when the button is clicked.
- checkedData: the data to be published once the button got checked.
- uncheckedData: the data to be published once the button got unchecked.

Subscribing properties **(not yet implemented)**:
- subscribeTopic: the button will display the string published to this topic

## Planned widgets:

### QLabel

Subscribing properties:
- subscribeTopic: the label will display the string published to this topic

### QLCDNumber

Subscribing properties:
- subscribeTopic: the label will display the number published to this topic

### QProgressbar

Subscribing properties:
- subscribeTopic: the label will display the number published to this topic

Subscribing properties:
- subscribeTopic: the button will display the string published to this topic

### QLineEdit/QTextEdit/QPlainTextEdit

Publishing properties:
- publishTopic: the topic/topic data field to publish data.
- publishEvent: possible values: editingFinished/textEdited. If the widget does not have this property then the data will be published on the textEdited signal (once the text contents changed).

Subscribing properties:
- subscribeTopic: the editor will display the string published to this topic. It is useful to set the readOnly property of the widget if this feature is used.

## QDateEdit / QDateTimeEdit

Publishing properties:
- publishTopic: the topic/topic data field to publish data
- publishEvent: possible values: editingFinished/dateChanged. If the widget does not have this property then the data will be published on the dateChanged signal (immediately once the date got changed).

Subscribing properties:
- subscribeTopic: the widget will display the time published to this topic. It is useful to set the readOnly property of the widget if this feature is used. The subscribed topic type needs to be std_msgs/Time.


###
