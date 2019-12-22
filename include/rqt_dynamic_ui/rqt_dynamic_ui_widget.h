#ifndef RQT_DYNAMIC_UI_WIDGET
#define RQT_DYNAMIC_UI_WIDGET

#include <QWidget>

#include <rqt_gui_cpp/plugin.h>


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

private slots:
    void on_patheditUIFile_pathChanged(const QString &path);

private:
    Ui::DynamicUIWidget *ui;

    QWidget *m_widget = nullptr;
};

#endif // RQT_DYNAMIC_UI_WIDGET
