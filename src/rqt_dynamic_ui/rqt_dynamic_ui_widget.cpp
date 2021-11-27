#include <rqt_dynamic_ui/rqt_dynamic_ui_widget.h>
#include "ui_rqt_dynamic_ui_widget.h"

#include <QtCore/QFile>
#include <QtUiTools/QUiLoader>
#include <QtWidgets/QAbstractSlider>
#include <QtWidgets/QAbstractSpinBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QMessageBox>

#include "ros/message_traits.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Char.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>

template <typename T>
void SerializeToByteArray(const T& msg, std::vector<uint8_t>& destination_buffer)
{
    const uint32_t length = ros::serialization::serializationLength(msg);
    destination_buffer.resize( length );
    //copy into your own buffer
    ros::serialization::OStream stream(destination_buffer.data(), length);
    ros::serialization::serialize(stream, msg);
}


DynamicUIWidget::DynamicUIWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DynamicUIWidget)
{
    ui->setupUi(this);
}

DynamicUIWidget::~DynamicUIWidget()
{
    delete ui;
    qDeleteAll(m_shapeShifters);
}

void DynamicUIWidget::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                   qt_gui_cpp::Settings &instance_settings) const
{
    Q_UNUSED(plugin_settings)
    instance_settings.setValue("lastUIFilePath", ui->patheditUiFile->path());
}

void DynamicUIWidget::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                      const qt_gui_cpp::Settings &instance_settings)
{
    Q_UNUSED(plugin_settings)
    QString uiPath = instance_settings.value("lastUIFilePath").toString();
    if (!uiPath.isEmpty())
        ui->patheditUiFile->setPath(uiPath);
}

void DynamicUIWidget::on_patheditUiFile_pathChanged(const QString &path)
{
    if (QFile::exists(path)) {
        QUiLoader loader;
        QFile file(path);
        if (file.open(QFile::ReadOnly)) {
            if (m_widget) {
                ui->gridLayoutLoadedUI->removeWidget(m_widget);
                m_widget->deleteLater();
            }

            m_widget = loader.load(&file, this);
            file.close();
            ui->gridLayoutLoadedUI->addWidget(m_widget);
            makeWidgetsRosConnections(m_widget);
            ui->toolButtonShowErrors->setVisible(!m_errors.isEmpty());
        }
    }
}

void DynamicUIWidget::makeWidgetsRosConnections(QObject *parentWidget)
{
    m_errors.clear();
    qDeleteAll(m_shapeShifters);
    m_shapeShifters.clear();
    for (auto children : parentWidget->children()) {
        if (QString(children->metaObject()->className()) == QStringLiteral("QSlider")
                || QString(children->metaObject()->className()) == QStringLiteral("QDial")) {
            makeSliderDialConnections(static_cast<QAbstractSlider*>(children));
        } else if (QString(children->metaObject()->className()) == QStringLiteral("QSpinBox")
                   || QString(children->metaObject()->className()) == QStringLiteral("QDoubleSpinBox")) {
            makeSpinBoxConnections(static_cast<QAbstractSpinBox*>(children));
        } else if (QString(children->metaObject()->className()) == QStringLiteral("QPushButton")
                   || QString(children->metaObject()->className()) == QStringLiteral("QToolButton")
                   || QString(children->metaObject()->className()) == QStringLiteral("QCheckBox")
                   || QString(children->metaObject()->className()) == QStringLiteral("QRadioButton")) {
            makeButtonConnections(static_cast<QAbstractButton*>(children));
        } else {
            makeWidgetsRosConnections(children);
        }
    }
}

void DynamicUIWidget::makeSliderDialConnections(QAbstractSlider *slider)
{
    if (slider->property("publishTopic").isValid()) {
        QString dataType = "Int32";
        if (slider->property("publishDataType").isValid()) {
            dataType = slider->property("publishDataType").toString();
            if (!isNumericDataType(dataType)) {
                addErrorString(tr("The %1 datatype is not supported to be published from a QDial/QSlider"));
                return;
            }
        }

        auto shifter = getShapeShifterForType(dataType);
        if (!shifter)
            return;
        m_shapeShifters.append(shifter);

        ros::Publisher publisher = shifter->advertise(m_nodeHandle, slider->property("publishTopic").toByteArray().constData(), 1);
        if (slider->property("publishEvent").toString() == "sliderReleased") {
            connect(slider, &QAbstractSlider::sliderReleased, this, [=] () {
                publishNumber<int>(slider->value(), dataType, shifter, publisher);
            });
        } else if (slider->property("publishEvent").toString() == "valueChanged"
                   || !slider->property("publishEvent").isValid()) {
            connect(slider, &QAbstractSlider::valueChanged, this, [=] (int value) {
                publishNumber<int>(value, dataType, shifter, publisher);
            });
        } else {
            addErrorString(tr("The '%1' publishEvent is not supported for QDial/QSlider!\n"
                              "Possible publishEvent properties: 'valueChanged', 'sliderReleased')")
                           .arg(slider->property("publishEvent").toString()));
        }
    }
}

void DynamicUIWidget::makeSpinBoxConnections(QAbstractSpinBox *spinBox)
{
    if (spinBox->property("publishTopic").isValid()) {
        QString dataType = (spinBox->metaObject()->className() == QStringLiteral("QSpinBox"))
                ? QStringLiteral("Int32")
                : QStringLiteral("Float32");
        if (spinBox->property("publishDataType").isValid()) {
            dataType = spinBox->property("publishDataType").toString();
            if (!isNumericDataType(dataType)) {
                addErrorString(tr("The %1 datatype is not supported to be published from a QSpinBox/QDoubleSpinBox"));
                return;
            }
        }

        auto shifter = getShapeShifterForType(dataType);
        if (!shifter)
            return;
        m_shapeShifters.append(shifter);

        ros::Publisher publisher = shifter->advertise(m_nodeHandle, spinBox->property("publishTopic").toByteArray().constData(), 1);
        if (spinBox->property("publishEvent").toString() == "editingFinished") {
            connect(spinBox, &QSpinBox::editingFinished, this, [=] () {
                if (spinBox->metaObject()->className() == QStringLiteral("QSpinBox"))
                    publishNumber<int>(static_cast<QSpinBox*>(spinBox)->value(), dataType, shifter, publisher);
                else
                    publishNumber<double>(static_cast<QDoubleSpinBox*>(spinBox)->value(), dataType, shifter, publisher);
            });
        } else if (spinBox->property("publishEvent").toString() == "valueChanged") {
            if (spinBox->metaObject()->className() == QStringLiteral("QSpinBox")) {
                connect(static_cast<QSpinBox*>(spinBox), QOverload<int>::of(&QSpinBox::valueChanged), this, [=] (int value) {
                    publishNumber<int>(value, dataType, shifter, publisher);
                });
            } else {
                connect(static_cast<QDoubleSpinBox*>(spinBox), QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=] (double value) {
                    publishNumber<double>(value, dataType, shifter, publisher);
                });
            }
        } else {
            addErrorString(tr("The '%1' publishEvent is not supported for QSpinBox/QDoubleSpinBox!\n"
                              "Possible publishEvent properties: 'valueChanged', 'editingFinished')"));
        }
    }
}


void DynamicUIWidget::makeButtonConnections(QAbstractButton *button)
{
    if (button->property("publishTopic").isValid()) {
        QString dataType = QStringLiteral("Bool");
        if (button->property("publishDataType").isValid()) {
            dataType = button->property("publishDataType").toString();
            if (!isValidStdDataType(dataType)) {
                addErrorString(tr("The %1 datatype is not supported to be published from a QPushButton/QToolButton/QRadioButton/QCheckBox"));
                return;
            }
        }

        auto shifter = getShapeShifterForType(dataType);
        if (!shifter)
            return;
        m_shapeShifters.append(shifter);

        ros::Publisher publisher = shifter->advertise(m_nodeHandle, button->property("publishTopic").toByteArray().constData(), 1);
        connect(button, &QAbstractButton::clicked, this, [=] () {
            if (button->property("publishCheckstate").isValid()) {
                if (isNumericDataType(dataType))
                    publishNumber<bool>(button->isChecked(), dataType, shifter, publisher);
                else if (dataType == "String")
                    publishString(button->isChecked() ? "1" : "0", dataType, shifter, publisher);
            } else if (button->property("publishDataOnClick").isValid()) {
                if (isNumericDataType(dataType))
                    publishNumber<qlonglong>(button->property("publishDataOnClick").toLongLong(), dataType, shifter, publisher);
                else if (dataType == "String")
                    publishString(button->property("publishDataOnClick").toString(), dataType, shifter, publisher);
            } else {
                const char* propertyName = button->isChecked() ? "checkedData" : "uncheckedData";
                if (isNumericDataType(dataType)) {
                    if (button->property(propertyName).isValid()) {
                        if (isFloatDataType(dataType))
                            publishNumber<double>(button->property(propertyName).toDouble(), dataType, shifter, publisher);
                        else {
                            if (isNumericDataType(dataType))
                                publishNumber<qlonglong>(button->property(propertyName).toLongLong(), dataType, shifter, publisher);
                        }
                    } else {
                        publishNumber<int>(button->isChecked() ? 1 : 0, dataType, shifter, publisher);
                    }
                } else {
                    // string or time or duration
                    if (dataType == "String")
                        publishString(button->property(propertyName).toString(), dataType, shifter, publisher);
                }
            }
        });
    }
}

topic_tools::ShapeShifter *DynamicUIWidget::getShapeShifterForType(const QString &dataType)
{
    topic_tools::ShapeShifter *ret = new topic_tools::ShapeShifter();

    if (dataType == "Bool") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::Bool>::value(), ros::message_traits::DataType<std_msgs::Bool>::value(), ros::message_traits::Definition<std_msgs::Bool>::value(), "");
    } else if (dataType == "Byte") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::Byte>::value(), ros::message_traits::DataType<std_msgs::Byte>::value(), ros::message_traits::Definition<std_msgs::Byte>::value(), "");
    } else if (dataType == "Char") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::Char>::value(), ros::message_traits::DataType<std_msgs::Char>::value(), ros::message_traits::Definition<std_msgs::Char>::value(), "");
    } else if (dataType == "Duration") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::Duration>::value(), ros::message_traits::DataType<std_msgs::Duration>::value(), ros::message_traits::Definition<std_msgs::Duration>::value(), "");
    } else if (dataType == "Float32") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::Float32>::value(), ros::message_traits::DataType<std_msgs::Float32>::value(), ros::message_traits::Definition<std_msgs::Float32>::value(), "");
    } else if (dataType == "Float64") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::Float64>::value(), ros::message_traits::DataType<std_msgs::Float64>::value(), ros::message_traits::Definition<std_msgs::Float64>::value(), "");
    } else if (dataType == "Int16") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::Int16>::value(), ros::message_traits::DataType<std_msgs::Int16>::value(), ros::message_traits::Definition<std_msgs::Int16>::value(), "");
    } else if (dataType == "Int32") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::Int32>::value(), ros::message_traits::DataType<std_msgs::Int32>::value(), ros::message_traits::Definition<std_msgs::Int32>::value(), "");
    } else if (dataType == "Int64") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::Int64>::value(), ros::message_traits::DataType<std_msgs::Int64>::value(), ros::message_traits::Definition<std_msgs::Int64>::value(), "");
    } else if (dataType == "Int8") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::Int8>::value(), ros::message_traits::DataType<std_msgs::Int8>::value(), ros::message_traits::Definition<std_msgs::Int8>::value(), "");
    } else if (dataType == "String") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::String>::value(), ros::message_traits::DataType<std_msgs::String>::value(), ros::message_traits::Definition<std_msgs::String>::value(), "");
    } else if (dataType == "Time") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::Time>::value(), ros::message_traits::DataType<std_msgs::Time>::value(), ros::message_traits::Definition<std_msgs::Time>::value(), "");
    } else if (dataType == "UInt16") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::UInt16>::value(), ros::message_traits::DataType<std_msgs::UInt16>::value(), ros::message_traits::Definition<std_msgs::UInt16>::value(), "");
    } else if (dataType == "UInt32") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::UInt32>::value(), ros::message_traits::DataType<std_msgs::UInt32>::value(), ros::message_traits::Definition<std_msgs::UInt32>::value(), "");
    } else if (dataType == "UInt64") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::UInt64>::value(), ros::message_traits::DataType<std_msgs::UInt64>::value(), ros::message_traits::Definition<std_msgs::UInt64>::value(), "");
    } else if (dataType == "UInt8") {
        ret->morph(ros::message_traits::MD5Sum<std_msgs::UInt8>::value(), ros::message_traits::DataType<std_msgs::UInt8>::value(), ros::message_traits::Definition<std_msgs::UInt8>::value(), "");
    } else {
        delete ret;
        ret = nullptr;
    }
    return ret;
}

bool DynamicUIWidget::isNumericDataType(const QString dataType)
{
    if (dataType == "Bool"
            || dataType == "Byte"
            || dataType == "Char"
            || dataType == "Float32"
            || dataType == "Float64"
            || dataType == "Int16"
            || dataType == "Int32"
            || dataType == "Int64"
            || dataType == "Int8"
            || dataType == "UInt16"
            || dataType == "UInt32"
            || dataType == "UInt64"
            || dataType == "UInt8") {
        return true;
    }
    return false;
}

bool DynamicUIWidget::isFloatDataType(const QString dataType)
{
    if (dataType == "Float32"
            || dataType == "Float64") {
        return true;
    }
    return false;
}

bool DynamicUIWidget::isValidStdDataType(const QString dataType)
{
    if (isNumericDataType(dataType))
        return true;
    if (dataType == "Duration"
            || dataType == "Time"
            || dataType == "String") {
        return true;
    }
    return false;
}

void DynamicUIWidget::addErrorString(const QString &string)
{
    if (m_errors.length())
        m_errors = m_errors.append("\n");
    m_errors = m_errors.append(string);
}

void DynamicUIWidget::publishString(const QString &value, const QString dataType, topic_tools::ShapeShifter *shifter, const ros::Publisher &publisher)
{
    std::vector<uint8_t> buffer;
    std_msgs::String outValue;
    outValue.data = value.toStdString();
    SerializeToByteArray(outValue, buffer);
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    shifter->read( stream );
    publisher.publish( *shifter );
}

template <typename T> void DynamicUIWidget::publishNumber(T value, const QString dataType,
                                                          topic_tools::ShapeShifter *shifter, const ros::Publisher &publisher)
{
    std::vector<uint8_t> buffer;
    if (dataType == "Bool") {
        std_msgs::Bool outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "Byte") {
        std_msgs::Byte outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "Char") {
        std_msgs::Char outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "Float32") {
        std_msgs::Float32 outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "Float64") {
        std_msgs::Float64 outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "Int16") {
        std_msgs::Int16 outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "Int32") {
        std_msgs::Int32 outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "Int64") {
        std_msgs::Int64 outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "Int8") {
        std_msgs::Int8 outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "String") {
        std_msgs::String outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "UInt16") {
        std_msgs::UInt16 outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "UInt32") {
        std_msgs::UInt32 outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "UInt64") {
        std_msgs::UInt64 outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else if (dataType == "UInt8") {
        std_msgs::UInt8 outValue;
        outValue.data = value;
        SerializeToByteArray(outValue, buffer);
    } else {
        return;
    }
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    shifter->read( stream );
    publisher.publish( *shifter );
}

void DynamicUIWidget::on_toolButtonReload_clicked()
{
    on_patheditUiFile_pathChanged(ui->patheditUiFile->path());
}

void DynamicUIWidget::on_toolButtonShowErrors_clicked()
{
    QMessageBox::warning(this,
                         tr("Errors"),
                         tr("The following errors have been found after parsing the provided UI file:\n%1")
                         .arg(m_errors));
}

