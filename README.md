### Dynamic UI rqt plugin

Have you ever wanted to create a small UI wihtout any business logic for publishing or visualizing some data from your ROS system?

Now you can create this without any coding using the rqt_dynamic_ui plugin!

## Usage workflow:

 - Design your user interface with QtDesigner or the Designer component of the QtCreator.
 - Add pubish/subscription properties to your widgets
 - Load the saved ui file to rqt with the rqt_dynamic_ui plugin
 - Enjoy your user interface!

## TODO 

 - Make it working!

## Planned to be supported widgets:

### QLabel

Subscribing properties:
- subscribeTopic: the label will display the string published to this topic

### QLCDNumber

Subscribing properties:
- subscribeTopic: the label will display the number published to this topic

### QProgressbar

Subscribing properties:
- subscribeTopic: the label will display the number published to this topic

### QPushButton/QToolButton

Publishing properties:
- publishTopic: the topic/topic data field to publish data upon the button interaction specified in the publishEvent
- publishData: the data to be published. Please make sure that the data type of this property matches the topic/topic field type
- publishCheckstate: if the widget has this property (and does not have checkedData or unCheckedData parameter) then a boolean value will be published to the publishTopic once the button is clicked. 
- checkedData: the data to be published once the button got checked. Please make sure that the data type of this property matches the topic/topic field type
- unCheckedData: the data to be published once the button got unchecked. Please make sure that the data type of this property matches the topic/topic field type

Subscribing properties:
- subscribeTopic: the button will display the string published to this topic

### QCheckBox/QRadioButton

Publishing properties:
- publishTopic: the topic/topic data field to publish data upon the button interaction specified in the publishEvent
- checkedData: the data to be published once the button got checked. Please make sure that the data type of this property matches the topic/topic field type.
- publishType: one data type from the supported numeric std_msgs. If this property is not present std_msgs::Bool is going to be used
- unCheckedData: the data to be published once the button got unchecked. Please make sure that the data type of this property matches the topic/topic field type.

Subscribing properties:
- subscribeTopic: the button will display the string published to this topic

### QLineEdit/QTextEdit/QPlainTextEdit

Publishing properties:
- publishTopic: the topic/topic data field to publish data.
- publishEvent: possible values: editingFinished/textEdited. If the widget does not have this property then the data will be published on the textEdited signal (once the text contents changed).

Subscribing properties:
- subscribeTopic: the editor will display the string published to this topic. It is useful to set the readOnly property of the widget if this feature is used.

### QSpinBox/QDoubleSpinBox

Publishing properties:
- publishTopic: the topic/topic data field to publish data
- publishEvent: possible values: editingFinished/valueChanged. If the widget does not have this property then the data will be published on the valueChanged signal (once the value is changed).

Subscribing properties:
- subscribeTopic: the widget will display the value published to this topic. It is useful to set the readOnly property of the widget if this feature is used.

### QDial/QSlider

Publishing properties:
- publishTopic: the topic/topic data field to publish data
- publishEvent: possible values: sliderReleased/valueChanged. If the widget does not have this property then the data will be published on the valueChanged signal (once the slider got moved).

Subscribing properties:
- subscribeTopic: the widget will display the value published to this topic. It is useful to set the readOnly property of the widget if this feature is used.

## QDateEdit / QDateTimeEdit

Publishing properties:
- publishTopic: the topic/topic data field to publish data
- publishEvent: possible values: editingFinished/dateChanged. If the widget does not have this property then the data will be published on the dateChanged signal (immediately once the date got changed).

Subscribing properties:
- subscribeTopic: the widget will display the time published to this topic. It is useful to set the readOnly property of the widget if this feature is used. The subscribed topic type needs to be std_msgs/Time.


###
