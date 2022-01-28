A simple tornado based webserver that can provide a configurable amout of packages in a ROS workspace via HTTP.

You can serve a specific package, in this case *ros_bt_py* by including the following code in your launch file:

```xml
  <!-- webserver -->
  <include file="$(find web_server)/launch/web_server.launch">
    <arg name="port" value="$(arg port)" />
    <arg name="cache_static_files" value="false" />
    <arg name="packages" value="[
        {
            package: ros_bt_py,
            directory: html,
            default_filename: editor.html
        },
      ]" />
  </include>
```

If you are running a single page webapp with a "Front Controller Pattern" design, like an angular routed webapp, you might want to enable the *rewrite_requests_to_default_filename* option for the particular package.

```xml
  <!-- webserver -->
  <include file="$(find web_server)/launch/web_server.launch">
    <arg name="port" value="$(arg port)" />
    <arg name="cache_static_files" value="false" />
    <arg name="packages" value="[
        {
            package: angular_web_app,
            directory: html,
            default_filename: editor.html,
            rewrite_requests_to_default_filename: true
        },
      ]" />
  </include>
```

If you only want to serve one package on the root url, you may be interested in the *serve_single_package_from_root* option, which will do exactly that:

```xml
  <!-- webserver -->
  <include file="$(find web_server)/launch/web_server.launch">
    <arg name="port" value="$(arg port)" />
    <arg name="cache_static_files" value="false" />
    <arg name="packages" value="[
        {
            package: ros_bt_py,
            directory: html,
            default_filename: editor.html,
        },
      ]" />
    <arg name="serve_single_package_from_root" value="true" />
  </include>
```