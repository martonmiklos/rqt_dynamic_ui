#include <rqt_dynamic_ui/rqt_dynamic_ui.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

namespace rqt_dynamic_ui {


DynamicUI::DynamicUI() :
    rqt_gui_cpp::Plugin()
{
    setObjectName("DynamicUI");
}

void DynamicUI::initPlugin(qt_gui_cpp::PluginContext &context)
{
    widget = new DynamicUIWidget();
    context.addWidget(widget);
}

void DynamicUI::shutdownPlugin()
{

}

void DynamicUI::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
{
    widget->saveSettings(plugin_settings, instance_settings);
}

void DynamicUI::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
{
    widget->restoreSettings(plugin_settings, instance_settings);
}

} // end namespace rqt_dynamic_ui

PLUGINLIB_EXPORT_CLASS(rqt_dynamic_ui::DynamicUI, rqt_gui_cpp::Plugin)
