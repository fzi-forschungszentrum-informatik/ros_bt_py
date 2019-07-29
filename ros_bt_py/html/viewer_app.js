var render = ReactDOM.render;

class ReadOnlyNamespaceSelect extends Component
{
  constructor(props)
  {
    super(props);

    this.state = {
      available_namespaces: []
    };

    this.updateAvailableNamespaces = this.updateAvailableNamespaces.bind(this);
    this.handleNamespaceChange = this.handleNamespaceChange.bind(this);
  }

  componentDidMount()
  {
    this.topicsForTypeClient = new ROSLIB.Service({
      ros: this.props.ros,
      name: '/rosapi/topics_for_type',
      serviceType: 'rosapi/TopicsForType'
    });

    this.updateAvailableNamespaces();
  }

  updateAvailableNamespaces()
  {
    this.topicsForTypeClient.callService(
      // Search for all Tree topics - we expect each BT node to
      // publish one of these, and also offer the corresponding
      // editing and runtime control services.
      new ROSLIB.ServiceRequest({
        type: 'ros_bt_py_msgs/Tree'
      }),
      function(response) {
        var namespaces = response.topics.map(
          // Chop off the topic name (but not the last slash), which leaves us with the BT
          // namespace
          x => x.substr(0, x.lastIndexOf('/')) + '/'
        );
        this.setState({
          available_namespaces: namespaces
        });
        if (this.props.currentNamespace === '' && namespaces.length > 0)
        {
          this.props.onNamespaceChange(namespaces[0]);
        }
      }.bind(this));
  }

  handleNamespaceChange(event)
  {
    this.props.onNamespaceChange(event.target.value);
  }

  render()
  {
    return (
      <Fragment>
        <div className="form-inline">
          <label className="ml-1">BT Namespace:
            <select className="custom-select ml-1"
                    value={this.props.currentNamespace}
                    onChange={this.handleNamespaceChange}>
              {
                this.state.available_namespaces.map(
                  x => (<option key={x}
                                value={x}>{x}</option>))
              }
            </select>
          </label>
        </div>
        <button type="button"
                className="btn btn-sm m-1"
                onClick={this.updateAvailableNamespaces}>
          <span aria-hidden="true" className="fas fa-sync" />
          <span className="sr-only">Refresh Namespaces</span>
        </button>
      </Fragment>
    );
  }
}

class App extends Component
{
  constructor(props)
  {
    super(props);

    var ros_uri = 'ws://localhost:9090';
    this.state = {
      bt_namespace: '',
      ros_uri: ros_uri,
      selected_tree: {
        is_subtree: false,
        name: ''
      },
      selected_edge: null,
      available_nodes: [],
      subtree_names: [],
      selected_node: null,
      selected_node_name: null,
      showDataGraph: true,
      last_tree_msg: null,
      // Can be 'nodelist' or 'editor'. The value decides whether the
      // "Add node" or "change node options" widget is shown.
      last_selection_source: 'nodelist',
      // The corresponding object from available_nodes for the
      // currently selected node. We need this because information
      // about OptionRefs isn't included in live nodes, but we need it
      // to edit options.
      selected_node_info: null,
      ros: new ROSLIB.Ros({
        url : ros_uri
      })
    };

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

    this.get_nodes_service = new ROSLIB.Service({
      ros: this.state.ros,
      name: props.bt_namespace + 'get_available_nodes',
      serviceType: 'ros_bt_py_msgs/GetAvailableNodes'
    });

    this.lastTreeUpdate = null;
    this.topicTimeoutID = null;
    this.newMsgDelay = 500;  // ms

    // Bind these here so this works as expected in callbacks
    this.getNodes = this.getNodes.bind(this);
    this.onError = this.onError.bind(this);
    this.onNodeListSelectionChange = this.onNodeListSelectionChange.bind(this);
    this.onEditorSelectionChange = this.onEditorSelectionChange.bind(this);
    this.onSelectedEdgeChange = this.onSelectedEdgeChange.bind(this);
    this.onTreeUpdate = this.onTreeUpdate.bind(this);
    this.onDebugUpdate = this.onDebugUpdate.bind(this);
    this.findPossibleParents = this.findPossibleParents.bind(this);
    this.onSelectedTreeChange = this.onSelectedTreeChange.bind(this);
    this.onNamespaceChange = this.onNamespaceChange.bind(this);
    this.updateTreeMsg = this.updateTreeMsg.bind(this);
  }

  onTreeUpdate(msg)
  {
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
      }
    }
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
      // Unsubscribe, then replace, topics
      this.tree_topic.unsubscribe(this.onTreeUpdate);
      this.debug_topic.unsubscribe(this.onDebugUpdate);

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

      // Subscribe again
      this.tree_topic.subscribe(this.onTreeUpdate);
      this.debug_topic.subscribe(this.onDebugUpdate);

      // Update GetAvailableNodes Service
      this.get_nodes_service = new ROSLIB.Service({
        ros: this.state.ros,
        name: namespace + 'get_available_nodes',
        serviceType: 'ros_bt_py_msgs/GetAvailableNodes'
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
  }

  componentWillUnmount()
  {
    this.tree_topic.unsubscribe(this.onTreeUpdate);
    this.debug_topic.unsubscribe(this.onDebugUpdate);
  }

  onError(error_message)
  {
    console.log(error_message);
  }

  onNodeListSelectionChange(new_selected_node)
  {
    this.setState({selected_node: new_selected_node,
                   selected_node_name: null,
                   last_selection_source: 'nodelist'});
  }

  onEditorSelectionChange(new_selected_node_name)
  {
    if (new_selected_node_name === null)
    {
      this.setState(
        {
          selected_node: null,
          selected_node_name: null,
          last_selection_source: 'editor',
        });
      return;
    }

    var new_selected_node = this.state.last_tree_msg.nodes.find(
      x => x.name === new_selected_node_name);;
    this.setState((prevState, props) => (
      {
        selected_node: new_selected_node,
        selected_node_name: new_selected_node_name,
        last_selection_source: 'editor',
        selected_node_info: prevState.available_nodes.find(
          x => (x.module === new_selected_node.module
                && x.node_class === new_selected_node.node_class))
      }
    ));
  }

  onSelectedEdgeChange(new_selected_edge)
  {
    this.setState({selected_edge: new_selected_edge});
  }

  render()
  {
    var selectedNodeComponent = null;

    if (this.state.selected_node === null)
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
          onError={this.onError}
        />);
    }


    return (
      <div className="container-fluid fill-parent d-flex flex-column">
        <div className="row">
          <div className="col d-flex">
            <ReadOnlyNamespaceSelect
              ros={this.state.ros}
              currentNamespace={this.state.bt_namespace}
              onNamespaceChange={this.onNamespaceChange}
              onError={this.onError}/>
            <SelectTree key={this.state.bt_namespace}
                        ros={this.state.ros}
                        bt_namespace={this.state.bt_namespace}
                        subtreeNames={this.state.subtree_names}
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
          </div>
        </div>
        <div className="row d-flex fill-parent pb-2">
          <D3BehaviorTreeEditor key={this.state.bt_namespace}
                                ros={this.state.ros}
                                bt_namespace={this.state.bt_namespace}
                                tree_message={this.state.last_tree_msg}
                                onSelectionChange={this.onEditorSelectionChange}
                                selectedNodeName={this.state.selected_node_name}
                                onSelectedEdgeChange={this.onSelectedEdgeChange}
                                showDataGraph={this.state.showDataGraph}
                                onError={this.onError}/>
        </div>
      </div>
    );
  }
}

render(<App />, document.getElementById("react-container"));