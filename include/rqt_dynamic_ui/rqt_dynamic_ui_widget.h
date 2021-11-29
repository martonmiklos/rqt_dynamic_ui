#pragma once

#include <QtWidgets/QWidget>

#include <rqt_gui_cpp/plugin.h>
#include <ros/publisher.h>
#include <ros/master.h>

#include "topic_tools/shape_shifter.h"

class QAbstractButton;
class QAbstractSlider;
class QAbstractSpinBox;

namespace Ui {
class DynamicUIWidget;
}

class DynamicUIWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DynamicUIWidget(QWidget *parent = nullptr);
    ~DynamicUIWidget();

    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private slots:
    void on_toolButtonReload_clicked();
    void uiCustomContextMenuRequested(const QPoint &pos);

    void on_patheditUiFile_pathChanged(const QString &path);
    void on_toolButtonShowErrors_clicked();

private:
    Ui::DynamicUIWidget *ui;
    ros::NodeHandle m_nodeHandle;

    QWidget *m_widget = nullptr;

    void makeWidgetsRosConnections(QObject *parentWidget);

    void makeSliderDialConnections(QAbstractSlider *slider);
    void makeSpinBoxConnections(QAbstractSpinBox *spinBox);
    void makeButtonConnections(QAbstractButton *button);

    topic_tools::ShapeShifter *getShapeShifterForType(const QString &dataType);

    static bool isNumericDataType(const QString dataType);
    static bool isFloatDataType(const QString dataType);
    static bool isValidStdDataType(const QString dataType);
    void addErrorString(const QString &string);
    QString m_errors;
    QList<topic_tools::ShapeShifter*> m_shapeShifters;

    template <typename T> void publishNumber(T value, const QString dataType, topic_tools::ShapeShifter *shifter, const ros::Publisher &publisher);
    void publishString(const QString &value, const QString dataType, topic_tools::ShapeShifter *shifter, const ros::Publisher &publisher);

    bool m_menuVisible = true;
    void setMenuVisible(bool visible);
};
