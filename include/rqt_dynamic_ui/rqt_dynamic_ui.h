#ifndef RQT_DYNAMIC_UI
#define RQT_DYNAMIC_UI

#include <rqt_gui_cpp/plugin.h>
#include <std_msgs/String.h>

#include <rqt_dynamic_ui/rqt_dynamic_ui_widget.h>

namespace rqt_dynamic_ui {

class DynamicUI : public rqt_gui_cpp::Plugin
{
public:
    DynamicUI();

    void initPlugin(qt_gui_cpp::PluginContext& context) override;
    void shutdownPlugin() override;

    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

private:
    DynamicUIWidget *widget = nullptr;
};

}

#endif // RQT_DYNAMIC_UI

