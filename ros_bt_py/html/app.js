var render = ReactDOM.render;
ReactModal.setAppElement("#react-container");

class App extends Component
{
  constructor(props)
  {
    super(props);

    var ros_uri = 'ws://' + window.location.hostname + ':9090';

    var parameters = window.location.search.substr(1);

    var params = {};
    var prmarr = parameters.split("&");
    for ( var i = 0; i < prmarr.length; i++) {
        var tmparr = prmarr[i].split("=");
        params[tmparr[0]] = tmparr[1];
    }

    if (params["ros_uri"])
    {
      ros_uri = params["ros_uri"];
    }

    var ros = new ROSLIB.Ros({
      url : ros_uri
    });

    this.state = {
      bt_namespace: '',
      cm_namespace: 'capability_manager/',
      ros_uri: ros_uri,
      selected_tree: {
        is_subtree: false,
        name: ''
      },
      error_history: [],
      error_history_sorting_asc: false,
      selected_edge: null,
      available_nodes: [],
      subtree_names: [],
      selected_node: null, // FIXME
      selected_node_names: [],
      copied_node: null,
      showDataGraph: true,
      dragging_node_list_item: null,
      last_tree_msg: null,
      // Can be 'nodelist' or 'editor'. The value decides whether the
      // "Add node" or "change node options" widget is shown.
      last_selection_source: 'nodelist',
      // The corresponding object from available_nodes for the
      // currently selected node. We need this because information
      // about OptionRefs isn't included in live nodes, but we need it
      // to edit options.
      selected_node_info: null,
      node_changed: false,
      ros: ros,
      skin: 'darkmode',
      copy_node: false,
      connected: false,
      publishing_subtrees: false,
      last_selected_package: '',
      show_file_modal: null,
    };

    ros.on("connection", function(e) {
      console.log("Connected to websocket");
      this.setState({connected: true});
    }.bind(this));

    ros.on("close", function(e) {
      this.setState({connected: false});
      console.log("Connection to websocket closed, reconnecting in 5s");
      setTimeout(function() {
        ros.connect(ros_uri);
      }, 5000);
    }.bind(this));

    this.tree_topic = new ROSLIB.Topic({
      ros : this.state.ros,
      name : this.state.bt_namespace + 'tree',
      messageType : 'ros_bt_py_msgs/Tree'
    });

    this.debug_topic = new ROSLIB.Topic({
      ros : this.state.ros,
      name: this.state.bt_namespace + 'debug/debug_info',
      messageType: 'ros_bt_py_msgs/DebugInfo'
    });

    this.messages_topic = new ROSLIB.Topic({
      ros : this.state.ros,
      name: this.state.bt_namespace + 'messages',
      messageType: 'ros_bt_py_msgs/Messages'
    });

    this.get_nodes_service = new ROSLIB.Service({
      ros: this.state.ros,
      name: this.state.bt_namespace + 'get_available_nodes',
      serviceType: 'ros_bt_py_msgs/GetAvailableNodes'
    });

    this.add_node_service = new ROSLIB.Service({
      ros: this.state.ros,
      name: this.state.bt_namespace + 'add_node',
      serviceType: 'ros_bt_py_msgs/AddNode'
    });

    this.remove_node_service = new ROSLIB.Service({
      ros: this.state.ros,
      name: this.state.bt_namespace + 'remove_node',
      serviceType: 'ros_bt_py_msgs/RemoveNode'
    });

    this.set_execution_mode_service = new ROSLIB.Service({
      ros: this.state.ros,
      name: this.state.bt_namespace + 'debug/set_execution_mode',
      serviceType: 'ros_bt_py_msgs/SetExecutionMode'
    });

    // topics needed for capability functionality
    this.packages_topic = new ROSLIB.Topic({
      ros : this.state.ros,
      name : this.state.bt_namespace + 'packages',
      messageType : 'ros_bt_py_msgs/Packages'
    });

    this.capabilities_topic = new ROSLIB.Topic({
      ros : this.state.ros,
      name : this.state.cm_namespace + 'capabilities',
      messageType : 'ros_ta_msgs/Capabilities'
    });

    this.lastTreeUpdate = null;
    this.topicTimeoutID = null;
    this.newMsgDelay = 500;  // ms

    // Bind these here so this works as expected in callbacks
    this.getNodes = this.getNodes.bind(this);
    this.onError = this.onError.bind(this);
    this.onClearErrors = this.onClearErrors.bind(this);
    this.onChangeErrorHistorySorting = this.onChangeErrorHistorySorting.bind(this);
    this.onNodeListSelectionChange = this.onNodeListSelectionChange.bind(this);
    this.onNodeListDragging = this.onNodeListDragging.bind(this);
    this.onChangeFileModal = this.onChangeFileModal.bind(this);
    this.check_dragging = this.check_dragging.bind(this);
    this.onNodeChanged = this.onNodeChanged.bind(this);
    this.onEditorSelectionChange = this.onEditorSelectionChange.bind(this);
    this.onMultipleSelectionChange = this.onMultipleSelectionChange.bind(this);
    this.onCapabilityPackageChange = this.onCapabilityPackageChange.bind(this);
    this.onSelectedEdgeChange = this.onSelectedEdgeChange.bind(this);
    this.onTreeUpdate = this.onTreeUpdate.bind(this);
    this.onDebugUpdate = this.onDebugUpdate.bind(this);
    this.onMessagesUpdate = this.onMessagesUpdate.bind(this);
    this.findPossibleParents = this.findPossibleParents.bind(this);
    this.onSelectedTreeChange = this.onSelectedTreeChange.bind(this);
    this.onNamespaceChange = this.onNamespaceChange.bind(this);
    this.updateTreeMsg = this.updateTreeMsg.bind(this);
    this.changeSkin = this.changeSkin.bind(this);
    this.changeCopyMode = this.changeCopyMode.bind(this);
    this.onPublishingSubtreesChange = this.onPublishingSubtreesChange.bind(this);
    this.onPackagesUpdate = this.onPackagesUpdate.bind(this);
    this.onCapabilitiesUpdate = this.onCapabilitiesUpdate.bind(this);
  }

  onTreeUpdate(msg)
  {
    if (this.state.publishing_subtrees && this.last_received_tree_msg && this.last_received_tree_msg.nodes)
    {
      var setup_and_shutdown = false;
      if (this.last_received_tree_msg.nodes.length != msg.nodes.length)
      {
        setup_and_shutdown = true;
      } else {
        for (var i = 0; i < msg.nodes.length; i++)
        {
          if (msg.nodes[i].module != this.last_received_tree_msg.nodes[i].module
              || msg.nodes[i].node_class != this.last_received_tree_msg.nodes[i].node_class
              || msg.nodes[i].name != this.last_received_tree_msg.nodes[i].name)
          {
            setup_and_shutdown = true;
          }
        }
      }
      if (setup_and_shutdown)
      {
        this.set_execution_mode_service.callService(
          new ROSLIB.ServiceRequest({
            single_step: false,
            publish_subtrees: true,
            collect_performance_data: false
          }),
          function(response) {
          }.bind(this));
      }
    }
    this.last_received_tree_msg = msg;
    if (!this.state.selected_tree.is_subtree)
    {
      this.updateTreeMsg(msg);
    }
  }

  onDebugUpdate(msg)
  {
    this.last_received_debug_msg = msg;
    this.setState({subtree_names: msg.subtree_states.map(x => x.name).sort()});
    if (this.state.selected_tree.is_subtree)
    {
      var selectedSubtree = msg.subtree_states
          .find(x => x.name === this.state.selected_tree.name);
      if (selectedSubtree)
      {
        this.updateTreeMsg(selectedSubtree);
      } else {
        this.onSelectedTreeChange(false, '');
      }
    }
  }

  onMessagesUpdate(msg)
  {
    console.log("received list of messages");
    this.messages = [];
    for (var i = 0; i < msg.messages.length; i++) {
      var components = msg.messages[i].msg.split("/");
      if (components.length == 2) {
        if (msg.messages[i].service)
        {
          this.messages.push({msg:components[0] + ".srv._" + components[1] + "." + components[1],
                              service: true});
          this.messages.push({msg:components[0] + ".srv._" + components[1] + "." + components[1] + "Request",
                              service: true});
          this.messages.push({msg:components[0] + ".srv._" + components[1] + "." + components[1] + "Response",
                              service: true});
        } else {
          this.messages.push({msg:components[0] + ".msg._" + components[1] + "." + components[1],
                              service: false});
        }
      }
    }
    var options = {
      shouldSort: true,
      threshold: 0.6,
      location: 0,
      distance: 100,
      maxPatternLength: 32,
      minMatchCharLength: 1,
      keys: [
        "msg"]
    };
    this.messagesFuse = new Fuse(this.messages, options);
  }

  onPackagesUpdate(msg)
  {
    console.log("received list of packages");
    this.last_received_packages_msg = msg;
    this.packages = msg.packages;

    var options = {
      shouldSort: true,
      threshold: 0.6,
      location: 0,
      distance: 100,
      maxPatternLength: 32,
      minMatchCharLength: 1,
      keys: [
        "package",
        "path"
      ]
    };
    this.packagesFuse = new Fuse(this.packages, options);
  }

  onCapabilitiesUpdate(msg)
  {
    console.log("received list of capabilities");
    this.last_received_capabilities_msg = msg;
    this.capabilities = [];
    for (var i = 0; i < msg.capabilities.length; i++) {
      var cap = {name:msg.capabilities[i].name, description:msg.capabilities[i].description, capability:msg.capabilities[i]};
      this.capabilities.push(cap);
    }
    var options = {
      shouldSort: true,
      threshold: 0.6,
      location: 0,
      distance: 100,
      maxPatternLength: 32,
      minMatchCharLength: 1,
      keys: [
        "name",
        "description"
      ]
    };
    this.capabilitiesFuse = new Fuse(this.capabilities, options);
  }

  changeSkin(skin)
  {
    this.setState({skin:skin});
  }

  changeCopyMode(mode)
  {
    if (mode)
    {
      // node copy/paste
      this.setState({copy_node: true});
    } else {
      // "normal" copy/paste for text
      this.setState({copy_node: false});
    }
  }

  onPublishingSubtreesChange(enable)
  {
    this.setState({publishing_subtrees: enable});
  }

  updateTreeMsg(msg)
  {
    // Clear any timers for previously received messages (see below)
    if (this.topicTimeoutID)
    {
      window.clearTimeout(this.topicTimeoutID);
      this.topicTimeoutID = null;
    }

    var now = Date.now();
    if (this.lastTreeUpdate === null || (now - this.lastTreeUpdate) > this.newMsgDelay)
    {
      this.setState({last_tree_msg: msg});
      this.lastTreeUpdate = now;
    }
    else
    {
      // if it hasn't been long enough since the last tree update,
      // schedule a retry so we don't drop a message.
      this.topicTimeoutID = window.setTimeout(
        function() {
          this.updateTreeMsg(msg);
      }.bind(this),
        this.newMsgDelay * 2);
    }
  }

  onSelectedTreeChange(is_subtree, name)
  {
    // Find the correct tree message (if any) to set for the new
    // selected tree
    var tree_msg = undefined;
    if (is_subtree)
    {
      tree_msg = this.last_received_debug_msg.subtree_states.find(x => x.name === name);
    }
    else
    {
      tree_msg = this.last_received_tree_msg;
    }

    if (tree_msg)
    {
      this.setState({
        selected_tree: {
          is_subtree: is_subtree,
          name: name
        },
        last_tree_msg: tree_msg
      });
      this.last_tree_update = Date.now();
    }
    else
    {
      this.setState({
        selected_tree: {
          is_subtree: is_subtree,
          name: name
        }
      });
    }
  }

  onNamespaceChange(namespace)
  {
    console.log('Namespace changed to: ', namespace);
    if (namespace !== this.state.bt_namespace)
    {
      this.setState({cm_namespace: namespace.replace('tree_node/', '') + 'capability_manager/'});

      // Unsubscribe, then replace, topics
      this.tree_topic.unsubscribe(this.onTreeUpdate);
      this.debug_topic.unsubscribe(this.onDebugUpdate);
      this.messages_topic.unsubscribe(this.onMessagesUpdate);
      this.packages_topic.unsubscribe(this.onPackagesUpdate);
      this.capabilities_topic.unsubscribe(this.onCapabilitiesUpdate);

      this.tree_topic = new ROSLIB.Topic({
        ros : this.state.ros,
        name : namespace + 'tree',
        messageType : 'ros_bt_py_msgs/Tree'
      });

      this.debug_topic = new ROSLIB.Topic({
        ros : this.state.ros,
        name: namespace + 'debug/debug_info',
        messageType: 'ros_bt_py_msgs/DebugInfo'
      });

      this.messages_topic = new ROSLIB.Topic({
        ros : this.state.ros,
        name: namespace + 'messages',
        messageType: 'ros_bt_py_msgs/Messages'
      });

      this.packages_topic = new ROSLIB.Topic({
        ros : this.state.ros,
        name : namespace + 'packages',
        messageType : 'ros_bt_py_msgs/Packages'
      });

      this.capabilities_topic = new ROSLIB.Topic({
        ros : this.state.ros,
        name : this.state.cm_namespace + 'capabilities',
        messageType : 'ros_ta_msgs/Capabilities'
      });

      // Subscribe again
      this.tree_topic.subscribe(this.onTreeUpdate);
      this.debug_topic.subscribe(this.onDebugUpdate);
      this.messages_topic.subscribe(this.onMessagesUpdate);
      this.packages_topic.subscribe(this.onPackagesUpdate);
      this.capabilities_topic.subscribe(this.onCapabilitiesUpdate);

      // Update GetAvailableNodes Service
      this.get_nodes_service = new ROSLIB.Service({
        ros: this.state.ros,
        name: namespace + 'get_available_nodes',
        serviceType: 'ros_bt_py_msgs/GetAvailableNodes'
      });

      this.add_node_service = new ROSLIB.Service({
        ros: this.state.ros,
        name: namespace + 'add_node',
        serviceType: 'ros_bt_py_msgs/AddNode'
      });

      this.remove_node_service = new ROSLIB.Service({
        ros: this.state.ros,
        name: namespace + 'remove_node',
        serviceType: 'ros_bt_py_msgs/RemoveNode'
      });

      this.set_execution_mode_service = new ROSLIB.Service({
        ros: this.state.ros,
        name: namespace + 'debug/set_execution_mode',
        serviceType: 'ros_bt_py_msgs/SetExecutionMode'
      });

      this.setState({bt_namespace: namespace});
    }
  }

  findPossibleParents()
  {
    if (this.state.last_tree_msg)
    {
      return this.state.last_tree_msg.nodes
        .filter(node => (node.max_children < 0 || node.child_names.length < node.max_children))
        .sort(function(a, b) {
          if (a.name < b.name) {
            return -1;
          }
          else if (a.name > b.name) {
            return 1;
          }
          else {
            return 0;
          }
        });
    }
    return [];
  }

  getNodes(package_name)
  {
    this.get_nodes_service.callService(
      new ROSLIB.ServiceRequest({
        node_modules: [package_name]
      }),
      function(response) {
        if (response.success) {
          this.setState({available_nodes: response.available_nodes});
        }
        else {
          this.onError('Failed to get list of nodes: ' + response.error_message);
        }
      }.bind(this));
  }

  componentDidMount()
  {
    this.tree_topic.subscribe(this.onTreeUpdate);
    this.debug_topic.subscribe(this.onDebugUpdate);
    this.messages_topic.subscribe(this.onMessagesUpdate);
    this.packages_topic.subscribe(this.onPackagesUpdate);
    this.capabilities_topic.subscribe(this.onCapabilitiesUpdate);

    document.body.addEventListener("keydown",function(e){
      if ( this.state.show_file_modal && e.keyCode == 27) // 27 = ESC
      {
        this.setState({show_file_modal: null});
      }
      if ( this.state.copy_node && e.keyCode == 67 && (e.ctrlKey || e.metaKey) ) { // 67 = KeyC
        if (this.state.selected_node_names.length > 1)
        {
          console.log("COPY/PASTE FOR MULTIPLE SELECTION NOT IMPLEMENTED YET");
          return;
        }
        this.setState({copied_node: this.state.selected_node});
      } else if ( this.state.copy_node && e.keyCode == 86 && (e.ctrlKey || e.metaKey) ) { // 86 = KeyV
        if (this.state.selected_node_names.length > 1)
        {
          console.log("COPY/PASTE FOR MULTIPLE SELECTION NOT IMPLEMENTED YET");
          return;
        }
        var parent = '';
        for (var i = 0; i < this.state.last_tree_msg.nodes.length; i++) {
          for (var j = 0; j < this.state.last_tree_msg.nodes[i].child_names.length; j++) {
            if(this.state.copied_node.name == this.state.last_tree_msg.nodes[i].child_names[j]) {
              parent = this.state.last_tree_msg.nodes[i].name;
              break;
            }
          }
        }

        this.add_node_service.callService(
          new ROSLIB.ServiceRequest({
            parent_name: parent,
            node: this.state.copied_node,
            allow_rename: true
          }),
          function(response) {
            if (response.success) {
              console.log('Added node to tree as ' + response.actual_node_name);
            }
            else {
              console.log('Failed to add node ' + this.state.name + ': '
                          + response.error_message);
            }
          }.bind(this));
      }
      if (this.state.copy_node && this.state.selected_node && e.keyCode == 46) { // 46 = Delete
        if (this.state.selected_node_names.length > 1)
        {
          console.log("DELETE FOR MULTIPLE SELECTION NOT IMPLEMENTED YET");
          return;
        }
        var remove_children = false;
        var remove_nodes_text = "Do you want to remove the selected node\"" + this.state.selected_node.name +"\"";

        if (e.shiftKey) {
          remove_children = true;
          remove_nodes_text += " and its children";
        }
        remove_nodes_text += "?";

        if (window.confirm(remove_nodes_text))
        {
          this.remove_node_service.callService(
            new ROSLIB.ServiceRequest({
              node_name: this.state.selected_node.name,
              remove_children: remove_children,
            }),
            function(response) {
              if (response.success) {
                console.log('Removed node from tree');
                this.onEditorSelectionChange(null);
              }
              else {
                console.log('Failed to remove node ' + response.error_message);
              }
            }.bind(this));
        }  
      }
    }.bind(this),false);
  }

  componentWillUnmount()
  {
    this.tree_topic.unsubscribe(this.onTreeUpdate);
    this.debug_topic.unsubscribe(this.onDebugUpdate);
    this.messages_topic.unsubscribe(this.onMessagesUpdate);
    this.packages_topic.unsubscribe(this.onPackagesUpdate);
    this.capabilities_topic.unsubscribe(this.onCapabilitiesUpdate);
  }

  onError(error_message)
  {
    this.setState({
      error_history: this.state.error_history.concat(
        {
          id: error_id(),
          time: Date.now(),
          text: error_message
        })
    });
    console.log(error_message);
  }

  onClearErrors()
  {
    this.setState({error_history:[]});
  }

  onChangeErrorHistorySorting(new_sorting)
  {
    this.setState({error_history_sorting_asc: new_sorting});
  }

  onNodeListSelectionChange(new_selected_node)
  {
    if (this.state.node_changed)
    {
      if(window.confirm("Are you sure you wish to discard all changes to the currently edited node?"))
      {
        // normal behavior, discard all entered data
        this.setState({node_changed: false});
      } else {
        // do not execute onNodeListSelectionChange and keep editing
        return;
      }
    }
    this.setState({selected_node: new_selected_node,
                   selected_node_names: [],
                   last_selection_source: 'nodelist'});
  }

  onNodeListDragging(dragging)
  {
    this.setState({dragging_node_list_item: dragging});
  }

  onChangeFileModal(mode)
  {
    this.setState({show_file_modal: mode});
  }

  check_dragging()
  {
    if (this.state.dragging_node_list_item)
    {
      this.setState({dragging_node_list_item: null});
    }
  }

  onMultipleSelectionChange(new_selected_node_names)
  {
    if (new_selected_node_names !== null)
    {
      this.setState(
        {
          selected_node: null,
          selected_node_names: new_selected_node_names,
          last_selection_source: 'multiple',
        });
      return;
    }
  }

  onCapabilityPackageChange(new_selected_package_name)
  {
    this.setState({last_selected_package: new_selected_package_name});
    return;
  }

  onEditorSelectionChange(new_selected_node_name)
  {
    if (this.state.node_changed && (new_selected_node_name === null || new_selected_node_name != this.state.selected_node_name))
    {
      if(window.confirm("Are you sure you wish to discard all changes to the currently edited node?"))
      {
        // normal behavior, discard all entered data
        this.setState({node_changed: false});
      } else {
        // do not execute onEditorSelectionChange and keep editing
        return;
      }
    }    

    if (new_selected_node_name === null)
    {
      this.setState(
        {
          selected_node: null,
          selected_node_names: [],
          last_selection_source: 'editor',
        });
      return;
    }

    var new_selected_node = this.state.last_tree_msg.nodes.find(x => x.name === new_selected_node_name);

    if (!new_selected_node)
    {
      this.setState(
        {
          selected_node: null,
          selected_node_names: [],
          last_selection_source: 'editor',
        });
      return;
    }

    this.setState((prevState, props) => (
      {
        copy_node: true,
        selected_node: new_selected_node,
        selected_node_names: [new_selected_node_name],
        last_selection_source: 'editor',
        selected_node_info: prevState.available_nodes.find(
          x => (x.module === new_selected_node.module
                && x.node_class === new_selected_node.node_class))
      }
    ));
  }

  onNodeChanged(state)
  {
    this.setState({node_changed: state});
  }

  onSelectedEdgeChange(new_selected_edge)
  {
    this.setState({selected_edge: new_selected_edge});
  }

  render()
  {
    var selectedNodeComponent = null;

    if (this.state.last_selection_source === 'multiple' && this.state.selected_node_names.length > 0)
    {
      selectedNodeComponent = (
        <MultipleSelection
          ros={this.state.ros}
          bt_namespace={this.state.bt_namespace}
          cm_namespace={this.state.cm_namespace}
          packages={this.packages}
          last_selected_package={this.state.last_selected_package}
          selectedNodeNames={this.state.selected_node_names}
          tree_message={this.state.last_tree_msg}
          packagesFuse={this.packagesFuse}
          capabilitiesFuse={this.capabilitiesFuse}
          onError={this.onError}
          onSelectionChange={this.onEditorSelectionChange}
          onMultipleSelectionChange={this.onMultipleSelectionChange}
          onSelectedEdgeChange={this.onSelectedEdgeChange}
          onCapabilityPackageChange={this.onCapabilityPackageChange}
        />);
    } else if (this.state.selected_node === null)
    {
      selectedNodeComponent = (
        <div className="d-flex flex-column">
          No Node Selected
        </div>
      );
    }
    else if (this.state.last_selection_source === 'nodelist')
    {
      selectedNodeComponent = (
        <NewNode
          ros={this.state.ros}
          bt_namespace={this.state.bt_namespace}
          key={
            this.state.bt_namespace
              + (this.state.selected_node ?
                 (this.state.selected_node.module
                  + this.state.selected_node.node_class)
                 :
                 '')
          }
          node={this.state.selected_node}
          parents={this.findPossibleParents()}
          messagesFuse={this.messagesFuse}
          capabilitiesFuse={this.capabilitiesFuse}
          onError={this.onError}
          onNodeChanged={this.onNodeChanged}
          changeCopyMode={this.changeCopyMode}
        />);
    }
    else if (this.state.last_selection_source === 'editor')
    {
      selectedNodeComponent = (
        <SelectedNode
          ros={this.state.ros}
          bt_namespace={this.state.bt_namespace}
          key={
            this.state.bt_namespace
              + (this.state.selected_node ?
                 this.state.selected_node.name
                 :
                 '')
          }
          node={this.state.selected_node}
          nodeInfo={this.state.selected_node_info}
          availableNodes={this.state.available_nodes}
          messagesFuse={this.messagesFuse}
          capabilitiesFuse={this.capabilitiesFuse}
          onError={this.onError}
          onNodeChanged={this.onNodeChanged}
          changeCopyMode={this.changeCopyMode}
          onEditorSelectionChange={this.onEditorSelectionChange}
        />);
    }

    var dragging_cursor = '';
    if(this.state.dragging_node_list_item)
    {
      dragging_cursor = 'cursor-grabbing'
    }

    return (
      <div onMouseUp={this.check_dragging} className={dragging_cursor}>
        <ReactModal key={this.state.bt_namespace}
                    isOpen={this.state.show_file_modal !== null}>
          <FileBrowser ros={this.state.ros}
                       bt_namespace={this.state.bt_namespace}
                       packagesFuse={this.packagesFuse}
                       onError={this.onError}
                       mode={this.state.show_file_modal}
                       onChangeFileModal={this.onChangeFileModal}/>
        </ReactModal>
        <ExecutionBar key={this.state.bt_namespace}
                      ros={this.state.ros}
                      connected={this.state.connected}
                      subtreeNames={this.state.subtree_names}
                      currentNamespace={this.state.bt_namespace}
                      tree_message={this.state.last_tree_msg}
                      onSelectedTreeChange={this.onSelectedTreeChange}
                      onNamespaceChange={this.onNamespaceChange}
                      onError={this.onError}
                      onPublishingSubtreesChange={this.onPublishingSubtreesChange}
                      onChangeFileModal={this.onChangeFileModal}/>

        <div className="container-fluid">
          <div className="row row-height">
            <div className="col scroll-col" id="nodelist_container">
              <NodeList key={this.state.bt_namespace}
                        availableNodes={this.state.available_nodes}
                        dragging_node_list_item={this.state.dragging_node_list_item}
                        getNodes={this.getNodes}
                        onSelectionChange={this.onNodeListSelectionChange}
                        onNodeListDragging={this.onNodeListDragging}/>
            </div>
            <div className="col-9 scroll-col" id="main_pane">
              <div className="container-fluid d-flex h-100 flex-column">
                <div className="row">
                  <div className="col d-flex">
                    <SelectTree key={this.state.bt_namespace}
                                ros={this.state.ros}
                                bt_namespace={this.state.bt_namespace}
                                subtreeNames={this.state.subtree_names}
                                selected_tree={this.state.selected_tree}
                                onSelectedTreeChange={this.onSelectedTreeChange}
                                onError={this.onError}/>
                    <button className="btn btn-primary m-1"
                            onClick={function() {
                              this.setState(
                                (prevstate, props) => ({showDataGraph: !prevstate.showDataGraph})
                              );
                            }.bind(this)
                                    }>
                      Toggle Data Graph
                    </button>
                    <Spacer />
                    <SelectEditorSkin changeSkin={this.changeSkin}/>
                  </div>
                </div>
                <div className="row edit_canvas h-100 pb-2">
                  <div className="col p-0">
                    <D3BehaviorTreeEditor key={this.state.bt_namespace}
                                          ros={this.state.ros}
                                          bt_namespace={this.state.bt_namespace}
                                          tree_message={this.state.last_tree_msg}
                                          subtreeNames={this.state.subtree_names}
                                          publishing_subtrees={this.state.publishing_subtrees}
                                          dragging_node_list_item={this.state.dragging_node_list_item}
                                          onSelectionChange={this.onEditorSelectionChange}
                                          onMultipleSelectionChange={this.onMultipleSelectionChange} // FIXME: DO NOT COMMIT
                                          selectedNodeNames={this.state.selected_node_names}         // FIXME: DO NOT COMMIT
                                          onSelectedEdgeChange={this.onSelectedEdgeChange}
                                          showDataGraph={this.state.showDataGraph}
                                          onSelectedTreeChange={this.onSelectedTreeChange}
                                          onNodeListDragging={this.onNodeListDragging}
                                          onError={this.onError}
                                          skin={this.state.skin}/>
                  </div>
                </div>
                <div className="row maxh50">
                  <div className="col pl-0">
                    {selectedNodeComponent}
                  </div>
                  <div className="col">
                    <div className="row pt-0 pl-0 pr-0">
                    {this.state.selected_edge ?
                     <BehaviorTreeEdge edge={this.state.selected_edge}
                                       key={this.state.bt_namespace}
                                       ros={this.state.ros}
                                       bt_namespace={this.state.bt_namespace}
                                       onSelectionChange={this.onEditorSelectionChange}
                                       unsetSelectedEdge={()=>this.setState({selected_edge: null})}
                                       onError={this.onError}/> :
                     <div className="d-flex flex-column">
                       No Edge Selected
                     </div>
                    }
                    </div>
                    <div  className="row output_log pl-0">
                      <ErrorHistory history={this.state.error_history}
                                    sorting_asc={this.state.error_history_sorting_asc}
                                    clearErrors={this.onClearErrors}
                                    changeSorting={this.onChangeErrorHistorySorting}/>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }
}
render(<App />, document.getElementById("react-container"));
