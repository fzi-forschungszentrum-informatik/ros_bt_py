
// import { Component } from 'preact';
// import { render } from 'preact';
// import { createRef } from 'preact';
var Component = React.Component;
var render = ReactDOM.render;
var createRef = React.createRef;
var Fragment = React.Fragment; //'x-fragment';

// uuid is used to assign unique IDs to tags so we can use labels properly
let idx = 0;
const uuid = () => idx++;

function typesCompatible(a, b)
{
  if (a.nodeName === b.nodeName) {
    return false;
  }

  if (a.kind === b.kind) {
    return false;
  }

  var from = a.kind === 'output' ? a : b;
  var to = a.kind === 'input' ? a : b;

  // object is compatible with anything
  if (to.type === "{\"py/type\": \"__builtin__.object\"}") {
    return true;
  }

  return prettyprint_type(from.type) === prettyprint_type(to.type);
}

var python_builtin_types = [
  'int',
  'float',
  'long',
  'str',
  'basestring',
  'unicode',
  'bool',
  'list',
  'dict',
  'set',
  'type'
];

function prettyprint_type(jsonpickled_type) {
  var json_type = JSON.parse(jsonpickled_type);
  if (json_type['py/type'] !== undefined)
  {
    // Remove the "builtin" prefix jsonpickle adds
    return json_type['py/type']
      .replace('__builtin__.', '')
      .replace(/^basestring$/, 'string')
      .replace(/^unicode$/, 'string')
      .replace(/^str$/, 'string');
  }

  // If the type doesn't have a py/type field, maybe it's an
  // OptionRef?
  if (json_type['py/object'] !== undefined &&
      json_type['py/object'] === 'ros_bt_py.node_config.OptionRef')
  {
    return 'OptionRef(' + json_type['option_key'] + ')';
  }

  return 'Unknown type object: ' + jsonpickled_type;
}

function getDefaultValue(typeName, options)
{
  if (typeName === 'type')
  {
    return {type: 'type',
            value: 'int'};
  }
  else if (typeName === 'int' || typeName === 'long')
  {
    return {type: 'int',
            value: 0};
  }
  else if (typeName === 'str' || typeName === 'basestring' || typeName === 'unicode'
           || typeName === 'string')
  {
    return {type: 'string',
            value: 'foo'};
  }
  else if (typeName === 'float')
  {
    return {type: 'float',
            value: 1.2};
  }
  else if (typeName === 'bool')
  {
    return {type: 'bool',
            value: true};
  }
  else if (typeName === 'list')
  {
    return {type: 'list',
            value: []};
  }
  else if (typeName === 'dict')
  {
    return {type: 'dict',
            value: {}};
  }
  else if (typeName.startsWith('OptionRef('))
  {
    var optionTypeName = typeName.substring(
        'OptionRef('.length, typeName.length - 1);
    var optionType = options.find(x => {
      return x.key === optionTypeName;
    });
    if (optionType)
    {
      return getDefaultValue(
        prettyprint_type(optionType.serialized_value));
    }
    else
    {
      return {
        type: 'unset_optionref',
        value: 'Ref to "' + optionTypeName + '"'
      };
    }
  }
  else
  {
    return {type: '__' + typeName,
            value: {}};
  }
}

// Get the distance between two sets of coordinates (expected to be
// arrays with 2 elements each)
function getDist(a, b)
{
  return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2));
}

function treeIsEditable(tree_msg)
{
  return tree_msg.state === "EDITABLE";
}

function selectIOGripper(vertex_selection, data)
{
  return vertex_selection
    .selectAll("." +
               data.data_kind.substring(0, data.data_kind.length - 1) +
               "-gripper-group")
    .filter(d => d.nodeName === data.node_name)
    .filter(d => d.key === data.data_key);
}


class NodeListItem extends Component {
  renderIOTable(nodedata_list, title) {
    // If there are no items in the list, don't generate any DOM
    // elements.
    if (nodedata_list.length == 0)
    {
      return null;
    }
    var rows = nodedata_list.map(data => {
      return (
        <tr key={title+data.key}>
          <td className="io_key">{data.key}</td>
          <td className="io_type text-muted pl-2">
            {prettyprint_type(data.serialized_value)}
          </td>
        </tr>);
    });

    return (
      <div className="io_values list-group-item">
        <h5>{title}</h5>
        <table><tbody>
            {rows}
        </tbody></table>
      </div>
    );
  };

  onClick(e, node) {
    this.props.onSelectionChange(this.props.node);
  }

  render() {
    return (
      <div className="border rounded p-2 mb-2"
           onClick={this.onClick.bind(this)}>
        <h4 className="node_class">{this.props.node.node_class}</h4>
        <h5 className="node_module text-muted">{this.props.node.module}</h5>
        <div>{
          'max_children: ' + (this.props.node.max_children >= 0 ? this.props.node.max_children : '∞')}</div>
        <div className="list-group">
          { this.renderIOTable(this.props.node.options, 'Options') }
          { this.renderIOTable(this.props.node.inputs, 'Inputs') }
          { this.renderIOTable(this.props.node.outputs, 'Outputs') }
        </div>
      </div>
    );
  };
}

class NodeList extends Component
{
  constructor(props)
  {
    super(props);

    this.state = {
      package_name: 'ros_bt_py.nodes.sequence'
    };

    this.handleChange = this.handleChange.bind(this);
  }

  componentDidMount()
  {
    this.props.getNodes('');
  }


  handleChange(e)
  {
    this.setState({package_name: e.target.value});
  }

  render()
  {
    var byName = function(a, b) {
      if (a.node_class < b.node_class)
      {
        return -1;
      }
      else if (a.node_class > b.node_class)
      {
        return 1;
      }

      return 0;
    };

    var moduleThenName = function(a, b) {
      if (a.module < b.module)
      {
        return -1;
      }
      else if (a.module > b.module)
      {
        return 1;
      }

      return byName(a, b);
    };

    var items = this.props.availableNodes
        .sort(byName)
//        .sort(moduleThenName)
        .map( (node) => {
          return (<NodeListItem node={node}
                           key={node.module + node.node_class}
                           onSelectionChange={this.props.onSelectionChange}/>);
    });
    return(
      <div className="available-nodes m-1">
        <div className="form-group">
          <button id="refresh"
                  className="btn btn-block btn-primary mt-2"
                  onClick={() => this.props.getNodes('')}>
            Refresh
          </button>
          <input type="text" id="package_name"
                 className="form-control mt-2"
                 value={this.state.package_name}
                 onChange={this.handleChange}/>
          <button id="load_package"
                  className="btn btn-block btn-primary mt-2"
                  onClick={() => this.props.getNodes(this.state.package_name)}>
            Load package
          </button>
        </div>
        <div className="vertical_list">
          {items}
        </div>
      </div>
    );
  }
}

class App extends Component
{
  constructor(props)
  {
    super(props);

    var ros_uri = 'ws://10.211.55.3:9090';
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
                   last_selection_source: 'nodelist'});
  }

  onEditorSelectionChange(new_selected_node_name)
  {
    var new_selected_node = this.last_received_tree_msg.nodes.find(
      x => x.name === new_selected_node_name);;
    this.setState((prevState, props) => (
      {
        selected_node: new_selected_node,
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

    if (this.state.last_selection_source === 'nodelist')
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
      <div>
        <ExecutionBar key={this.state.bt_namespace}
                      ros={this.state.ros}
                      subtreeNames={this.state.subtree_names}
                      currentNamespace={this.state.bt_namespace}
                      onSelectedTreeChange={this.onSelectedTreeChange}
                      onNamespaceChange={this.onNamespaceChange}
                      onError={this.onError}/>

        <div className="container-fluid">
          <div className="row row-height">
            <div className="col scroll-col" id="nodelist_container">
              <NodeList key={this.state.bt_namespace}
                        availableNodes={this.state.available_nodes}
                        getNodes={this.getNodes}
                        onSelectionChange={this.onNodeListSelectionChange}/>
            </div>
            <div className="col-9 scroll-col" id="main_pane">
              <div className="container-fluid">
                <div className="row">
                  <div className="col d-flex">
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
                <div className="row edit_canvas pb-2">
                  <div className="col p-0">
                    <D3BehaviorTreeEditor key={this.state.bt_namespace}
                                          ros={this.state.ros}
                                          bt_namespace={this.state.bt_namespace}
                                          tree_message={this.state.last_tree_msg}
                                          onSelectionChange={this.onEditorSelectionChange}
                                          onSelectedEdgeChange={this.onSelectedEdgeChange}
                                          showDataGraph={this.state.showDataGraph}
                                          onError={this.onError}/>
                  </div>
                </div>
                <div className="row">
                  <div className="col pl-0">
                    {selectedNodeComponent}
                  </div>
                  <div className="col pr-0">
                    {this.state.selected_edge &&
                     <BehaviorTreeEdge edge={this.state.selected_edge}
                                       key={this.state.bt_namespace}
                                       ros={this.state.ros}
                                       bt_namespace={this.state.bt_namespace}
                                       onSelectionChange={this.onEditorSelectionChange}
                                       onError={this.onError}/>}
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

function ExecutionBar(props)
{
  return (
    <header id="header" className="d-flex flex-column flex-md-row align-items-center control-bar">
      <NamespaceSelect
        ros={props.ros}
        currentNamespace={props.currentNamespace}
        onNamespaceChange={props.onNamespaceChange}
        onError={props.onError}/>
      <DebugControls
        ros={props.ros}
        bt_namespace={props.currentNamespace}
        onError={props.onError}/>
      <TickControls
        ros={props.ros}
        bt_namespace={props.currentNamespace}
        onError={props.onError}/>
    </header>
  );
}

class NamespaceSelect extends Component
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
    this.servicesForTypeClient = new ROSLIB.Service({
      ros: this.props.ros,
      name: '/rosapi/services_for_type',
      serviceType: 'rosapi/ServicesForType'
    });

    this.updateAvailableNamespaces();
  }

  updateAvailableNamespaces()
  {
    this.servicesForTypeClient.callService(
      // Search for all Tree topics - we expect each BT node to
      // publish one of these, and also offer the corresponding
      // editing and runtime control services.
      new ROSLIB.ServiceRequest({
        type: 'ros_bt_py_msgs/AddNode'
      }),
      function(response) {
        var namespaces = response.services.map(
          x => x.substr(0, x.lastIndexOf('/')) + '/'
        );
        this.setState({
          // Chop off the topic name (but not the last slash), which leaves us with the BT
          // namespace
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
                    defaultValue={this.props.currentNamespace}
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

class SelectTree extends Component
{
  constructor(props)
  {
    super(props);

    this.onChange = this.onChange.bind(this);
  }

  onChange(event)
  {
    var value = parseInt(event.target.value);
    if (value < 0)
    {
      this.props.onSelectedTreeChange(
        /*is_subtree=*/ false,
        /*name=*/ ''); // no name needed, there's only one not-subtree
    }
    else
    {
      this.props.onSelectedTreeChange(
        /*is_subtree=*/ true,
        /*name=*/ this.props.subtreeNames[value]);
    }
  }

  render()
  {
    return (
      <div>
        <label className="form-inline m-1">Tree:
          <select className="custom-select ml-1"
                  defaultValue="main"
                  onChange={this.onChange}>
            <option value="-1">Main Tree</option>
            <optgroup label="Subtrees">
              {
                this.props.subtreeNames.map(
                  (name, index) =>  (<option key={name} value={index}>{name}</option>))
              }
            </optgroup>
          </select>
        </label>
      </div>
    );
  }
}

class TickControls extends Component
{
  constructor(props)
  {
    super(props);

  }
  componentDidMount()
  {
    this.tick_service = new ROSLIB.Service({
      ros: this.props.ros,
      name: this.props.bt_namespace + 'control_tree_execution',
      serviceType: 'ros_bt_py_msgs/ControlTreeExecution'
    });

  }
  controlExec(command)
  {
    this.tick_service.callService(
      new ROSLIB.ServiceRequest({
        // TICK_ONCE = 1
        // TICK_PERIODICALLY = 2
        // TICK_UNTIL_RESULT = 3
        // STOP = 4
        // RESET = 5
        // SHUTDOWN = 6
        command: command
      }),
      function(response) {
        if (response.success) {
          console.log('called ControlTreeExecution service successfully');
        }
        else {
          this.props.onError(response.error_message);
        }
      }.bind(this));
  }

  render()
  {
    return (
      <Fragment>
        <button onClick={this.controlExec.bind(this, 1)}
                className="btn btn-primary m-1">
          Tick Once
        </button>
        <button onClick={this.controlExec.bind(this, 2)}
                className="btn btn-primary m-1">
          Tick Periodically
        </button>
        <button onClick={this.controlExec.bind(this, 4)}
                className="btn btn-primary m-1">
          Stop
        </button>
        <button onClick={this.controlExec.bind(this, 5)}
                className="btn btn-primary m-1">
          Reset
        </button>
        <button onClick={this.controlExec.bind(this, 6)}
                className="btn btn-primary m-1">
          Shutdown
        </button>
      </Fragment>
    );
  }
}

class DebugControls extends Component
{
  constructor(props)
  {
    super(props);

    this.state = {
      debugging: false,
      publishing_subtrees: false
    };

    this.debug_settings_sub = new ROSLIB.Topic({
      ros: props.ros,
      name: props.bt_namespace + 'debug/debug_settings',
      messageType: 'ros_bt_py_msgs/DebugSettings'
    });

    this.set_execution_mode_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'debug/set_execution_mode',
      serviceType: 'ros_bt_py_msgs/SetExecutionMode'
    });

    this.step_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'debug/continue',
      serviceType: 'ros_bt_py_msgs/Continue'
    });

    this.debugCheckID = 'debug_' + uuid();
    this.publishSubtreesID = 'publish_subtrees_' + uuid();

    this.onNewDebugSettings = this.onNewDebugSettings.bind(this);
    this.onClickStep = this.onClickStep.bind(this);
    this.handleDebugChange = this.handleDebugChange.bind(this);
    this.handlePubSubtreesChange = this.handlePubSubtreesChange.bind(this);
  }

  componentDidMount()
  {
    this.debug_settings_sub.subscribe(this.onNewDebugSettings);
  }

  componentWillUnmount()
  {
    this.debug_settings_sub.unsubscribe(this.onNewDebugSettings);
  }

  onNewDebugSettings(msg)
  {
    this.setState({debugging: msg.single_step,
                   publishing_subtrees: msg.publish_subtrees});
  }

  onClickStep()
  {
    this.step_service.callService(
      new ROSLIB.ServiceRequest({}),
      function(response) {
        if (response.success) {
          console.log('stepped successfully');
        }
        else {
          this.props.onError(response.error_message);
        }
      }.bind(this));
  }

  handleDebugChange(event)
  {
    var enable = event.target.checked;
    this.set_execution_mode_service.callService(
      new ROSLIB.ServiceRequest({
        single_step: enable,
        publish_subtrees: this.state.publishing_subtrees,
        collect_performance_data: true
      }),
      function(response) {
        if (enable) {
          console.log('enabled stepping');
        }
        else {
          console.log('disabled stepping');
        }
      }.bind(this));
    this.setState({debugging: enable});
  }

  handlePubSubtreesChange(event)
  {
    var enable = event.target.checked;
    this.set_execution_mode_service.callService(
      new ROSLIB.ServiceRequest({
        single_step: this.state.debugging,
        publish_subtrees: enable,
        collect_performance_data: true
      }),
      function(response) {
        if (enable) {
          console.log('enabled stepping');
        }
        else {
          console.log('disabled stepping');
        }
      }.bind(this));
    this.setState({publishing_subtrees: enable});
  }

  render()
  {
    return (
      <Fragment>
        <div className="custom-control custom-checkbox m-1">
          <input type="checkbox"
                 id={this.debugCheckID}
                 className="custom-control-input"
                 checked={this.state.debugging}
                 onChange={this.handleDebugChange} />
          <label className="custom-control-label"
                 htmlFor={this.debugCheckID}>Debug</label>
        </div>
        <div className="custom-control custom-checkbox m-1">
          <input type="checkbox"
                 id={this.publishSubtreesID}
                 className="custom-control-input"
                 checked={this.state.publishing_subtrees}
                 onChange={this.handlePubSubtreesChange} />
          <label className="custom-control-label"
                 htmlFor={this.publishSubtreesID}>Publish Subtrees</label>
        </div>
        <button onClick={this.onClickStep}
                className="btn btn-primary m-1">
          Step
        </button>
      </Fragment>
    );
  }
}

class D3BehaviorTreeEditor extends Component
{
  constructor(props)
  {
    super(props);

    this.spacing = 80;

    this.min_node_drag_distance=15;

    this.io_gripper_spacing = 10;
    this.io_gripper_size = 15;
    this.max_io_gripper_size = 15;

    this.nextWiringSource = null;
    this.nextWiringTarget = null;

    this.draggedNode = null;
    this.dragging = false;

    // ### Pan and zoom stuff ###
    this.zoomObject = null;
    // Begin panning if the mouse is less than this away from the
    // viewport's edge
    this.dragPanBoundary = 50;
    this.panIntervalID = null;
    this.panDirection = [0.0, 0.0];
    this.panRate = 30;
    this.panPerFrame = 10.0;

    this.wire_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'wire_data',
      serviceType: 'ros_bt_py_msgs/WireNodeData'
    });

    this.unwire_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'unwire_data',
      serviceType: 'ros_bt_py_msgs/UnwireNodeData'
    });

    this.move_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'move_node',
      serviceType: 'ros_bt_py_msgs/MoveNode'
    });

    this.replace_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'replace_node',
      serviceType: 'ros_bt_py_msgs/ReplaceNode'
    });

    this.svg_ref = createRef();
    this.viewport_ref = createRef();

    this.dragPanTimerHandler = this.dragPanTimerHandler.bind(this);
    this.resetView = this.resetView.bind(this);
    this.getIOCoords = this.getIOCoords.bind(this);
    this.getIOCoordsFromNode = this.getIOCoordsFromNode.bind(this);
  }

  componentDidMount()
  {
    // Give Viewport pan / zoom
    var viewport = d3.select(this.viewport_ref.current);
    var width = viewport.node().getBoundingClientRect().width;
    var height = viewport.node().getBoundingClientRect().height;

    this.zoomObject = d3.zoom();
    var container = d3.select(this.svg_ref.current);

    viewport
      .call(this.zoomObject.scaleExtent([0.3, 1.0]).on("zoom", function () {
        container.attr("transform", d3.event.transform);
      }))
      .call(this.zoomObject.translateTo, 0.0, (height * 0.5) - 10.0);

    // Add Mousemove listener to pan viewport while draggins
    viewport.on("mousemove.pan_if_drag", this.canvasMousemovePanHandler.bind(this));
  }

  render()
  {
    return (
      <svg id="editor_viewport"
           ref={this.viewport_ref}
           className="reactive-svg">
        <g id="container" ref={this.svg_ref}>
          { // order is important here - SVG draws things in the order
            // they appear in the markup!
          }
          <g className="edges"/>
          <g className="vertices"/>
          { // Data Graph should be above the node graph
            // (since it can be toggled on and off)
          }
          <g className="data_graph">
            { // We want the edges below the vertices here because that looks nicer
            }
            <g className="data_edges" />
            <g className="data_vertices" />
          </g>
          { // This is for the targets that appear when the user is dragging
            // a node to reposition it.
            // Obviously, these should be above anything else!
          }
          <g className="drop_targets"
             visibility="hidden" />
        </g>
      </svg>
    );
  }

  componentDidUpdate(prevProps, prevState)
  {
    if (this.props.tree_message !== prevProps.tree_message)
    {
      this.drawEverything(this.props.tree_message);

      // Disable all interaction (except for zooming and panning) when
      // the tree isn't editable
      if (treeIsEditable(this.props.tree_message))
      {

      }
    }
    // Hide or show data graph
    if (this.props.showDataGraph)
    {
      d3.select(this.svg_ref.current).select(".data_graph").attr("visibility", "visible");
    }
    else
    {
      d3.select(this.svg_ref.current).select(".data_graph").attr("visibility", "hidden");
    }

  }

  drawEverything(tree_msg)
  {
    if (!tree_msg)
    {
      return;
    };

    var onlyKeyAndType = nodeData => ({
      key: nodeData.key,
      serialized_type: nodeData.serialized_type
    });
    // Trim the serialized data values from the node data - we won't
    // render them, so don't clutter the DOM with the data
    var trimmed_nodes = tree_msg.nodes.map(function(node) {
      return {
        node_class: node.node_class,
        module: node.module,
        name: node.name,
        state: node.state,
        child_names: node.child_names,
        options: node.options.map(onlyKeyAndType),
        inputs: node.inputs.map(onlyKeyAndType),
        outputs: node.outputs.map(onlyKeyAndType)
      };
    });

    var forest_root = {
      name: "__forest_root",
      child_names: [],
      inputs: [],
      outputs: [],
      options: []
    };

    if (trimmed_nodes.findIndex(x => x.name === "__forest_root") < 0)
    {
      trimmed_nodes.push(forest_root);
    }
    // Update the visual tree
    var parents = {};
    var node_dict = {};
    // Find parents for all nodes once
    (function(){
      for (var i in trimmed_nodes) {
        var node = trimmed_nodes[i];
        node_dict[node.name] = node;
        for (var j in node.child_names) {
          parents[node.child_names[j]] = node.name;
        }
      }
    })();

    var root = d3
        .stratify()
        .id(function(node) {
          return node.name;
        })
        .parentId(function(node) {
          // undefined if it has no parent - does that break the layout?
          if (node.name in parents) {
            return parents[node.name];
          } else if (node.name === forest_root.name) {
            return undefined;
          } else {
            forest_root.child_names.push(node.name);
            return forest_root.name;
          }
        })(trimmed_nodes);

    root.sort(function(a, b) {
      if (a.depth !== b.depth) {
        return b.depth - a.depth;
      }
      while (a.parent !== b.parent) {
        a = a.parent;
        b = b.parent;
      }
      var child_list = a.parent.data.child_names;
      return (
        child_list.findIndex(x => x === a.data.name) -
          child_list.findIndex(x => x === b.data.name)
      );
    });


    var svg = d3.select(this.svg_ref.current);


    var container = d3.select(this.viewport_ref.current);
    var width = container.attr("width"),
        height = container.attr("height");


    var g_edge = svg.select("g.edges");
    var g_vertex = svg.selectAll("g.vertices");
    var g_data = svg.selectAll("g.data_graph");
    var g_droptargets = svg.selectAll("g.drop_targets");

    var node = g_vertex
        .selectAll(".node")
        .data(root.descendants()
              .filter(node => node.id !== forest_root.name),
              function(node) {
                return node.id;
              });

    node.exit().remove();

    node = node
      .enter()
      .call(this.drawNodes.bind(this))
      .merge(node)
      .selectAll(".btnode")
      .data(x => [x], x => x.id)
      .call(this.updateNodes.bind(this));

    // TODO(nberg): Find a way to get rid of this - it's here because
    // the DOM changes in updateNodes take a while to actually happen,
    // and layoutNodes needs getBoundingClientRect information...
    window.setTimeout(
      () =>
        {
          this.layoutNodes(svg, width, height, root);

          this.drawDropTargets();

          this.drawDataGraph(g_data, node.data(), tree_msg.data_wirings);
        }, 100);

    //console.log(root);
  }

  drawNodes(selection)
  {
    selection.each(function(d) {
      d._entering = true;
      d._show = true;
      d.x = 0;
      d.y = 0;
    });

    var that = this;
    var fo = selection.append('foreignObject')
        .attr("class", function(d) {
          return "node" + (d.children ? " node--internal" : " node--leaf");
        })
        .on("click", this.nodeClickHandler.bind(this))
        .on("mousedown", function(d, index, group) {
          that.nodeMousedownHandler(d, this);
        });

    var div = fo
        .append("xhtml:body")
        .attr("class", "btnode p-2")
        .style("min-height", d => {
          // We need to ensure a minimum height, in case the node body
          // would otherwise be shorter than the number of grippers
          // requires.
          var inputs = d.data.inputs || [];
          var outputs = d.data.outputs || [];
          var max_num_grippers = Math.max(inputs.length, outputs.length);
          return ((this.io_gripper_size + this.io_gripper_spacing) * max_num_grippers) + 'px';
        });
  }

  updateNodes(selection)
  {
    // Update name
    var title = selection.selectAll(".node_name").data(function(d) {
      return [d];
    });
    title = title.enter().append("h4").attr("class", "node_name").merge(title);
    title.html(function(d) {
      return d.id;
    });

    var className = selection.selectAll(".class_name").data(function(d) {
      return [d];
    });
    className = className.enter().append("h5").attr("class", "class_name").merge(className);
    className.html(function(d) {
      return d.data.node_class;
    });
  }

  fillTables (tbody)
  {
    var rows = tbody.selectAll(".node_data")
        .data(function(d) {
          return d.value;
        }, d => d.key);

    rows.exit().remove();
    // create new rows
    rows = rows.enter().append("tr").attr("class", "node_data").merge(rows);

    var keys = rows.selectAll(".key").data(d => [d.key]);
    keys.exit().remove();
    keys = keys.enter().append("td").attr("class", "key").merge(keys);
    keys.text(d => d);

    var values = rows.selectAll(".value").data(d => [d.serialized_value]);
    values.exit().remove();
    values = values.enter().append("td").attr("class", "value").merge(values);
    values.text(d => d);
  }

  layoutNodes(svg, width, height, root)
  {
    var g_edge = svg.select("g.edges");
    var g_vertex = svg.selectAll("g.vertices");

    var nodes_without_forest_root = root.descendants()
        .filter(node => node.id !== '__forest_root');
    // k is the zoom level - we need to apply this to the values we get
    // from getBoundingClientRect, or we get fun scaling effects.
    var zoom = d3.zoomTransform(d3.select(this.viewport_ref.current).node()).k;

    // Find the maximum size of all the nodes, for layout purposes
    var max_size = [0,0];
    var max_height_by_depth = Array(root.height + 1).fill(0.0);
    g_vertex.selectAll('.btnode')
      .data(nodes_without_forest_root,
            x => x.id)
      .each(function(d, index){
        var rect = this.getBoundingClientRect();
        rect.x /= zoom;
        rect.y /= zoom;
        rect.width /= zoom;
        rect.height /= zoom;
        d._size = rect;
        max_height_by_depth[d.depth] = Math.max(max_height_by_depth[d.depth], rect.height);
        this.parentElement.setAttribute('width', rect.width);
        this.parentElement.setAttribute('height', rect.height);
      });

    var tree_size = [width - max_size[0], height - (40 + max_size[1])];

    var tree = d3.flextree()
        .nodeSize(function(node) {
          if (node._size)
          {
            return [node._size.width + this.spacing,
                    max_height_by_depth[node.depth] + this.spacing];
          }
          else
          {
            return [1,1];
          }
        }.bind(this))
    (root);

    // Move new nodes to their starting positions
    g_vertex.selectAll(".node")
      .filter(d => d._entering)
      .attr("transform", function(d) {
        // Start at parent position
        var p = this.findExistingParent(d);
        return "translate(" + Math.round(p.x) + "," + Math.round(p.y) + ") scale(0.1)";
      }.bind(this));

    var link = g_edge.selectAll(".link")
        .data(tree.links()
              .filter(x=>x.source.id !== '__forest_root' && x.target.id !== '__forest_root'),
              function(d) { return '' + d.source.id + d.target.id; });
    link.exit().remove();

    link = link
      .enter().append("path")
      .attr("class", "link")
      .attr("d", d3.linkVertical()
            .source(function(d) {
              var parent = this.findExistingParent(d.source);
              return [Math.round(parent.x), Math.round(parent.y + parent._size.height)];
            }.bind(this))
            .target(function(d) {
              var parent = this.findExistingParent(d.target);
              return [Math.round(parent.x), Math.round(parent.y)];
            }.bind(this)))
      .merge(link);

    g_vertex.selectAll(".node").each(function(d) {
      d._entering = false;
    });

    link.transition()
      .duration(250).
      attr("d", d3.linkVertical()
           .source(function(d) {
             return [Math.round(d.source.x), Math.round(d.source.y + d.source._size.height)];
           })
           .target(function(d) {
             return [Math.round(d.target.x), Math.round(d.target.y)];
           }));


    // new selection, now with the elements we just added with enter()
    // above
    var node = g_vertex.selectAll(".node")
      .data(root.descendants(), function(node) {return node.id;});

    var t = d3.transition()
        .duration(250);
    node.transition(t)
      .attr("transform", function(d) {
        // animate to actual position
        return "translate(" + Math.round(d.x - d._size.width / 2.0) + "," + Math.round(d.y) + ") scale(1.0)";
      });
        node
      .selectAll(".btnode")
      .transition(t)
      .ease(d3.easeQuad)
    // Update color based on node state
      .style("border-color", function(d) {
        switch (d.data.state) {
        case "RUNNING": {
          return "#ffc107";
        }
        case "IDLE":{
          return "#007bff";
        }
        case "SUCCEEDED": {
          return "#28a745";
        }
        case "FAILED": {
          return "#dc3545";
        }
        case "DEBUG_PRE_TICK":
        case "DEBUG_POST_TICK":
        case "DEBUG_TICK": {
          return "#17a2b8";
        }
        case "SHUTDOWN": {
          return "#7c1e27";
        }
        case "UNINITIALIZED":
        default: {
          return "#4E5666";
        }
        };
      });
  }

  findExistingParent(d)
  {
    while (d._entering && d.parent && d.parent._size) {
      d = d.parent;
    }
    return d;
  }

  // DATA GRAPH HELPERS
  getGripperSize(height, inputs, outputs)
  {
    return Math.min(
      this.max_io_gripper_size,
      // +1 to ensure nice padding on the bottom
      outputs != 0 ?
        ((height - ((outputs + 1) * this.io_gripper_spacing)) / outputs)
        :
        this.max_io_gripper_size,
      inputs != 0 ?
        ((height - ((inputs + 1) * this.io_gripper_spacing)) / inputs)
        :
        this.max_io_gripper_size);
  }

  getGripperCoords(index, right, gripper_size, node_width)
  {
    return {
      x: 0.5 * node_width - (right ? 0.0 : gripper_size + node_width),
      y: this.io_gripper_spacing + (index * (this.io_gripper_spacing + gripper_size))
    };
  }

  drawDropTargets()
  {
    var g_droptargets = d3.select(this.svg_ref.current).select("g.drop_targets");

    // We can't really assign keys to the targets, so remove them all :/
    g_droptargets.selectAll(".drop_target").remove();

    // For each node, decide what kinds of drop targets to add.
    //
    // Possible drop targets are:
    //
    // Neighbors (left/right):
    // Insert the dropped node into the parent's list of children
    // before or after this node.
    // Only available if the parent doesn't have its maximum number of children yet!
    //
    // Replace (bounding rect of the node itself):
    // Replace this node with the dropped node.
    //
    // Below:
    // Add the dropped node as this node's child - only available if there's 0 children!
    //
    // Above:
    // Add the dropped node as this node's parent, taking over its old position.
    // Note that this is realized by two service calls:
    // 1. Move this node to be dropped node's child
    // 2. Move dropped node to this node's position
    d3.select(this.svg_ref.current).select("g.vertices").selectAll(".node").each(
      function(d)
      {
        // No drop options at the root of the tree (__forest_root)!
        if (!d.parent ) {
          return;
        }

        var my_index = d.parent.children.findIndex(x => x.data.name == d.data.name);

        // Only add the "insert before" target for the first node.
        // Without this if, we'd get overlapping drop targets for
        // this.after and next_child.before
        if (my_index == 0)
        {
          g_droptargets
            .append("rect")
            .attr("class", "drop_target")
            .attr("transform",
                  "translate(" + (d.x - this.spacing - (d._size.width * 0.5)) + ","
                  + d.y + ")")
            .attr("width", this.spacing)
            .attr("height", d._size.height)
            .datum({
              position: my_index,  // insert before this node
              replace: false,
              data: d.parent.data
            });
        }
        g_droptargets
          .append("rect")
          .attr("class", "drop_target")
          .attr("transform",
                "translate(" + (d.x + (d._size.width * 0.5)) + ","
                + d.y + ")")
          .attr("width", this.spacing)
          .attr("height", d._size.height)
          .datum({
            position: my_index + 1,  // insert after this node
            replace: false,
            data: d.parent.data
          });

        g_droptargets
          .append("rect")
          .attr("class", "drop_target")
          .attr("transform",
                "translate(" + (d.x - (d._size.width * 0.5)) + ","
                + d.y + ")")
          .attr("width", d._size.width)
          .attr("height", d._size.height)
          .datum({
            position: -1,
            replace: true,  // replace this node
            data: d.data
          });

        g_droptargets
          .append("rect")
          .attr("class", "drop_target")
          .attr("transform",
                "translate(" + (d.x - (d._size.width * 0.5)) + ","
                + (d.y - (this.spacing * 0.5)) + ")")
          .attr("width", d._size.width)
          .attr("height", this.spacing * 0.45)
          .datum({
            // replace the node at the given index from data, and take
            // it as our own child
            position: my_index,
            replace: true,
            data: d.parent.data
          });

        var child_names = d.data.child_names || [];
        var max_children = d.data.max_children || -1;
        // If max_children is either -1 or >0, we can add a child
        if (max_children != 0 && child_names.length == 0)
        {
          g_droptargets
            .append("rect")
            .attr("class", "drop_target")
            .attr("transform",
                  "translate(" + (d.x - (d._size.width * 0.5)) + ","
                  + (d.y + d._size.height) + ")")
            .attr("width", d._size.width)
            .attr("height", this.spacing * 0.45)
            .datum({
              // Add as first child of this node
              position: 0,
              replace: false,
              data: d.data
            });
        }
      }.bind(this));

    var that = this;
    g_droptargets.selectAll(".drop_target")
      .attr("opacity", 0.2)
      .on("mouseover", function(d, index, group)
          {
            that.dropTargetDefaultMouseoverHandler(this, d);
          })
      .on("mouseout", function(d, index, group)
          {
            that.dropTargetDefaultMouseoutHandler(this, d);
          });
  }

  drawDataGraph(g_data, data, wirings)
  {
    var edges = g_data.select("g.data_edges");
    var vertices = g_data.select("g.data_vertices");

    var input_vertex_data = [];
    var output_vertex_data = [];
    data.forEach(function(x) {
      Array.prototype.push.apply(
        input_vertex_data,
        x.data.inputs.map(
          function(input) {
            var datum = this.getIOCoordsFromNode(
              x,
              input.key,
              "inputs",
              /*centered=*/false);
            datum.nodeName = x.data.name;
            datum.key = input.key;
            datum.kind = "input";
            datum.type = input.serialized_type;
            return datum;
          }.bind(this)));

      Array.prototype.push.apply(
        output_vertex_data,
        x.data.outputs.map(
          function(output) {
            var datum = this.getIOCoordsFromNode(
              x,
              output.key,
              "outputs",
              /*centered=*/false);
            datum.nodeName = x.data.name;
            datum.key = output.key;
            datum.kind = "output";
            datum.type = output.serialized_type;
            return datum;
          }.bind(this)));
    }.bind(this));

    this.drawDataVerts(vertices, input_vertex_data, output_vertex_data);

    this.drawDataEdges(
      edges,
      wirings.map(function(wiring) {
        var start = this.getIOCoords(data,
                                     wiring.source.node_name,
                                     wiring.source.data_kind,
                                     wiring.source.data_key,
                                     /*centered=*/true);

        var two = {
          x : start.x ,
          y : start.y - 2
        };

        if (wiring.source.data_kind === "inputs")
        {
          two.x = two.x - 10;
        }
        else if (wiring.source.data_kind === "outputs")
        {
          two.x = two.x + 10;
        }
        else
        {
          // two.y = two.y - 10;
        }

        var target = this.getIOCoords(data,
                                      wiring.target.node_name,
                                      wiring.target.data_kind,
                                      wiring.target.data_key,
                                      /*centered=*/true);

        var three = {
          x : target.x ,
          y : target.y - 2
        };

        if (wiring.target.data_kind === "inputs")
        {
          three.x = three.x - 10;
        }
        else if (wiring.target.data_kind === "outputs")
        {
          three.x = three.x + 10;
        }
        else
        {
          // three.y = three.y - 10;
        }

        return {
          source: {
            node_name: wiring.source.node_name,
            data_kind: wiring.source.data_kind,
            data_key: wiring.source.data_key
          },
          target: {
            node_name: wiring.target.node_name,
            data_kind: wiring.target.data_kind,
            data_key: wiring.target.data_key
          },
          points: [
            start,
            two,
            three,
            target
          ]};
      }.bind(this)));
  }

  getIOCoords(node_data,
              node_name,
              data_kind,
              data_key,
              centered)
  {
    var node = node_data.find(d => d.data.name === node_name);
    return this.getIOCoordsFromNode(node, data_key, data_kind, centered);
  }

  getIOCoordsFromNode(
    node,
    data_key,
    data_kind,
    centered)
  {
    centered = centered || false;

    if (!node)
    {
      // Shouldn't really happen...
      return {x: 0, y: 0, gripperSize: 0};
    }

    var coords = null;
    var inputs = node.inputs || [];
    var outputs = node.outputs || [];

    if (data_kind === 'inputs')
    {
      coords = this.getGripperCoords(
        node.data.inputs.findIndex(x => x.key === data_key) || 0,
        /*right=*/false,
        this.io_gripper_size,
        node._size.width);
    }
    else if (data_kind === 'outputs')
    {
      coords = this.getGripperCoords(
        node.data.outputs.findIndex(x => x.key === data_key) || 0,
        /*right=*/true,
        this.io_gripper_size,
        node._size.width);
    }
    else
    {
      // For things that are neither inputs nor outputs, just draw a
      // line to the center of the node
      coords = {
        x: 0.5 * node._size.width,
        y: 0.5 * node._size.height
      };
    }

    if (centered)
    {
      coords.x += this.io_gripper_size * 0.5;
      coords.y += this.io_gripper_size * 0.5;
    }
    return {
      x: node.x + coords.x,
      y: node.y + coords.y,
      gripperSize: this.io_gripper_size
    };
  }

  drawDataEdges(edge_selection, edge_data)
  {
    var link = edge_selection.selectAll(".data-link").data(
      edge_data,
      d => JSON.stringify(d.source) + JSON.stringify(d.target));
    link.exit().remove();

    link = link
      .enter()
      .append("path")
      .attr("class", "data-link")
      .on("click", this.DataEdgeDefaultClickHandler.bind(this))
      .on("mouseover", this.DataEdgeDefaultMouseoverHandler)
      .on("mouseout", this.DataEdgeDefaultMouseoutHandler)
      .merge(link);

    link
      .transition()
      .attr("d", function(d) {
        var lineGen = d3.line()
            .x(d => d.x)
            .y(d => d.y)
            .curve(d3.curveCatmullRom.alpha(0.9));
        return lineGen(d.points);
      });
  }

  drawDataVerts(vertex_selection, input_vertex_data, output_vertex_data)
  {
    var groups = vertex_selection.selectAll(".gripper-group").data(
      input_vertex_data.concat(output_vertex_data),
      d => d.nodeName + d.kind + d.key);
    groups.exit().remove();

    groups = groups
      .enter()
      .append("g")
      .attr("class", d => "gripper-group " + d.kind + "-gripper-group")
      .on("mouseover.highlight", this.IOGroupDefaultMouseoverHandler)
      .on("mouseout.highlight", this.IOGroupDefaultMouseoutHandler)
      .merge(groups);

    groups
      .transition()
      .attr("transform", function(d) {
        return "translate(" + Math.round(d.x) + ", " + Math.round(d.y) + ")";
      });

    var grippers = groups.selectAll(".gripper").data(d=> [d]);
    grippers.exit().remove();

    grippers = grippers
      .enter()
      .append("rect")
      .attr("class", d => "gripper " + d.kind + "-gripper")
      .attr("width", d => d.gripperSize)
      .attr("height", d => d.gripperSize)
      .on("mousedown", this.IOGripperMousedownHandler.bind(this))
      .merge(grippers);

    var labels = groups.selectAll(".label").data(d=> [d]);
    labels.exit().remove();

    labels = labels
      .enter()
      .append("text")
      .attr("class", "label")
      .attr("text-anchor", d => d.kind === "input" ? "end" : "start")
      .attr("dominant-baseline", "middle")
      .attr("visibility", "hidden")
      .attr("dx", d => {
        if (d.kind === "input") {
          return Math.round(-5);
        }
        else if (d.kind === "output") {
          return Math.round(d.gripperSize + 5);
        }
        return 0;
      })
      .attr("dy", d => Math.round(0.5 * d.gripperSize))
      .merge(labels);
    labels.text(d => d.key);
  }

  IOGroupDefaultMouseoverHandler(d, index, group)
  {
    d3.select(this).classed("data-hover", true)
      .selectAll(".label")
      .attr("visibility", "visible");
  }

  IOGroupDefaultMouseoutHandler(d, index, group)
  {
    d3.select(this).classed("data-hover", false)
      .selectAll(".label")
      .attr("visibility", "hidden");
  }

  IOGroupDraggingMouseoverHandler(d, index, group)
  {
    if (d.kind === "input")
    {
      this.nextWiringTarget = d;
    }
    else if (d.kind === "output") {
      this.nextWiringSource = d;
    }
  }

  IOGroupDraggingMouseoutHandler(d, index, group)
  {
    if (d.kind === "input")
    {
      this.nextWiringTarget = null;
    }
    else if (d.kind === "output") {
      this.nextWiringSource = null;
    }
  }

  DataEdgeDefaultMouseoverHandler(d, index, group)
  {
    var vertex_selection = d3.select(this.parentNode.parentNode)
        .select("g.data_vertices");
    d3.select(this).classed("data-hover", true);

    // select source gripper
    selectIOGripper(vertex_selection, d.source)
      .dispatch("mouseover");

    // select target gripper
    selectIOGripper(vertex_selection, d.target)
      .dispatch("mouseover");
  }

  DataEdgeDefaultMouseoutHandler(d, index, group)
  {
    var vertex_selection = d3.select(this.parentNode.parentNode)
        .select("g.data_vertices");

    d3.select(this).classed("data-hover", false);

    // deselect source gripper
    selectIOGripper(vertex_selection, d.source)
      .dispatch("mouseout");

    // deselect target gripper
    selectIOGripper(vertex_selection, d.target)
      .dispatch("mouseout");
  }

  DataEdgeDefaultClickHandler(d, index, group)
  {
    this.props.onSelectedEdgeChange(d);
  }

  IOGripperMousedownHandler(datum, index, group)
  {
    if ((d3.event.buttons & 1) != 1)
    {
      return;
    }
    // Remove mouseover / out listeners from all gripper-groups, then add new ones
    var svg_sel = d3.select(this.svg_ref.current);
    var io_grippers = svg_sel.selectAll(".gripper-group");

    io_grippers
      .on("mouseover", null)
      .on("mouseout", null);

    // Remove mouseover listener from data edges so we don't
    // accidentally send a mouseover event to a gripper while crossing
    // an edge connected to it
    svg_sel.selectAll('.data-link')
      .on("mouseover", null)
      .on("mouseout", null);

    // Hide this gripper's label
    svg_sel.selectAll(".label").datum(datum).attr("visibility", "hidden");

    // Save the datum to use in the wire request later
    if (datum.kind === "input")
    {
      this.nextWiringTarget = datum;
    }
    else if (datum.kind === "output") {
      this.nextWiringSource = datum;
    }

    // Also save the current mouse position (relative to the viewport
    // <g> tag)
    this.dragStartPos = d3.mouse(this.svg_ref.current);

    // Give compatible IOs a new listener and highlight them
    io_grippers
      .filter(d => typesCompatible(d, datum))
      .classed("compatible", true)
      .on("mouseover.drag",
          this.IOGroupDraggingMouseoverHandler.bind(this))
      .on("mouseout.drag",
          this.IOGroupDraggingMouseoutHandler.bind(this))
      .selectAll(".label")
      .attr("visibility", "visible");

    this.dragging = true;
    // Give the canvas a move and mouseup handler
    d3.select(this.viewport_ref.current)
    // Remove any handlers for node dragging
      .on("mousemove.drag_node", null)
      .on("mouseup.drag_node", null)
      .on("mousemove.drag_io", this.canvasIOMoveHandler.bind(this))
      .on("mouseup.drag_io", this.canvasIOUpHandler.bind(this));

    d3.event.preventDefault();
    d3.event.stopPropagation();
  }

  canvasMousemovePanHandler(d, index, group)
  {
    // Don't do anything unless we're currently dragging something
    if (!this.dragging){
      if (this.panIntervalID !== null)
      {
        window.clearInterval(this.panIntervalID);
        this.panDirection = [];
      }

      return;
    }

    var viewport = d3.select(this.viewport_ref.current);
    var width = viewport.node().getBoundingClientRect().width;
    var height = viewport.node().getBoundingClientRect().height;

    var mouseCoords = d3.mouse(this.viewport_ref.current);

    var panNeeded = false;
    this.panDirection = [0.0, 0.0];

    if (mouseCoords[0] < this.dragPanBoundary) {
      // Left edge -> scroll right
      panNeeded = true;
      // 1 if mouse is at the left edge, 0 if it is dragPanBoundary pixels away
      var leftEdgeCloseness = (this.dragPanBoundary - mouseCoords[0]) / this.dragPanBoundary;
      this.panDirection[0] = this.panPerFrame * leftEdgeCloseness;
    } else if (mouseCoords[0] > (width - this.dragPanBoundary)) {
      // Right edge -> scroll left
      panNeeded = true;
      // 1 if mouse is at the right edge, 0 if it is dragPanBoundary pixels away
      var rightEdgeCloseness = (this.dragPanBoundary - (width - mouseCoords[0]))
          / this.dragPanBoundary;
      this.panDirection[0] = -1.0 * this.panPerFrame * rightEdgeCloseness;
    }
    if (mouseCoords[1] < this.dragPanBoundary) {
      // Up -> scroll down
      panNeeded = true;
      // 1 if mouse is at the top edge, 0 if it is dragPanBoundary pixels away
      var topEdgeCloseness = (this.dragPanBoundary - mouseCoords[1]) / this.dragPanBoundary;
      this.panDirection[1] = this.panPerFrame * topEdgeCloseness;
    } else if (mouseCoords[1] > (height - this.dragPanBoundary)) {
      // Down -> scroll up
      panNeeded = true;
      // 1 if mouse is at the bottom edge, 0 if it is dragPanBoundary pixels away
      var botEdgeCloseness = (this.dragPanBoundary - (height - mouseCoords[1]))
          / this.dragPanBoundary;
      this.panDirection[1] = -1.0 * this.panPerFrame * botEdgeCloseness;
    }

    if (!panNeeded && this.panIntervalID !== null)
    {
      window.clearInterval(this.panIntervalID);
      this.panIntervalID = null;
      return;
    }

    if (this.panIntervalID === null)
    {
      // Start the interval for the panning animation, at panRate Hz
      this.panIntervalID = window.setInterval(this.dragPanTimerHandler, 1000.0 / this.panRate);
    }
  }

  dragPanTimerHandler()
  {
    if (!this.dragging && this.panIntervalID)
    {
      window.clearInterval(this.panIntervalID);
      this.panIntervalID = null;
      return;
    }

    d3.select(this.viewport_ref.current)
      .call(this.zoomObject.translateBy, this.panDirection[0], this.panDirection[1]);
  }

  canvasIOMoveHandler(d, index, group)
  {
    if ((d3.event.buttons & 1) === 0)
    {
      this.canvasIOUpHandler(d, index, group);
      return;
    }

    var drawingLine = d3
        .select(this.svg_ref.current)
        .selectAll(".drawing-indicator")
        .data(
          [{
            start: this.dragStartPos,
            end: d3.mouse(this.svg_ref.current)
          }],
          /*key=*/d => JSON.stringify(d.start));

    drawingLine.exit().remove();
    drawingLine = drawingLine
      .enter()
      .append("path")
      .attr("class", "drawing-indicator")
      .merge(drawingLine);

    drawingLine
      .attr("d", data => d3.line()([data.start, data.end]));
    d3.event.preventDefault();
  }

  canvasIOUpHandler(d, index, group)
  {
    this.dragging = false;

    if (this.nextWiringSource && this.nextWiringTarget)
    {
      this.wire_service.callService(
        new ROSLIB.ServiceRequest({
          wirings: [
            {
              source: {
                node_name: this.nextWiringSource.nodeName,
                data_kind: this.nextWiringSource.kind + "s", // should be inputs!
                data_key: this.nextWiringSource.key
              },
              target: {
                node_name: this.nextWiringTarget.nodeName,
                data_kind: this.nextWiringTarget.kind + "s", // should be outputs!
                data_key: this.nextWiringTarget.key
              }
            }]
        }),
        function(response) {
          if (response.success)
          {
            console.log("Successfully wired data!");
          }
          else
          {
            this.props.onError("Failed to wire data " + this.nextWiringSource +
                               " to " + this.nextWiringTarget + ": " + JSON.stringify(response));
          }
        }.bind(this));
    }

    // Either way, we're done dragging, so restore the old handlers

    // Remove mouseover / out listeners from all gripper-groups, then
    // add back the default ones
    var io_grippers = d3.select(this.svg_ref.current).selectAll(".gripper-group");
    io_grippers
      .on("mouseover", null)
      .on("mouseout", null)
    // Also remove this class again
      .classed("data-hover", false)
      .classed("compatible", false);

    io_grippers
    // Remove the drag listeners
      .on("mouseover.drag", null)
      .on("mouseout.drag", null)
    // And hide the labels again
      .selectAll(".label")
      .attr("visibility", "hidden");


    d3.select(this.svg_ref.current).selectAll('.data-link')
      .on("mouseover", this.DataEdgeDefaultMouseoverHandler)
      .on("mouseout", this.DataEdgeDefaultMouseoutHandler);

    // Also remove listeners from the background
    d3.select(this.viewport_ref.current)
      .on("mousemove.drag_io", null)
      .on("mouseup.drag_io", null);

    // Remove the drawing line from the DOM
    d3.select(this.viewport_ref.current).selectAll(".drawing-indicator").data([])
      .exit().remove();

    // Finally, remove these:
    this.nextWiringSource = null;
    this.nextWiringTarget = null;
  }

  canvasNodeDragMoveHandler()
  {
    if (this.draggedNode === null)
    {
      return;
    }
    var newly_dragging = (
      !this.dragging
        && getDist(d3.mouse(this.draggedNode.domObject),
                   this.draggedNode.startCoords) > this.min_node_drag_distance);

    if (newly_dragging)
    {
      this.dragging = true;
      // Hide all drop targets that would lead to appending the dragged
      // node to its own subtree
      var d = this.draggedNode.data;
      var parentName = d.parent ? d.parent.data.name || '' : '';
      var my_index = d.parent.children.findIndex(x => x.data.name == d.data.name);

      var svg = d3.select(this.svg_ref.current);

      var all_children = d.descendants().map(x => x.data.name);

      var g_droptargets = svg.select("g.drop_targets");

      g_droptargets.attr("visibility", "visible");

      g_droptargets.selectAll(".drop_target")
      // First ensure all drop targets are visible
        .attr("visibility", "visible")
      // Now hide those that belong to descendants of the node we're dragging
        .filter(
          x => all_children.indexOf(x.data.name) >= 0
            || (x.data.name === parentName
                && (x.position == my_index
                    || x.position == my_index + 1)))
        .attr("visibility", "hidden");
    }

    if ((d3.event.buttons & 1) === 0)
    {
      this.canvasNodeDragUpHandler(d, index, group);
    }

    d3.event.preventDefault();
    d3.event.stopPropagation();
  }

  canvasNodeDragUpHandler(d, index, group)
  {
    if (this.dragging)
    {
      if (this.nodeDropTarget)
      {
        // Calculate the final index to move the dropped node to.
        //
        // If the target is in the same parent node, and after the
        // current index of the dropped node, subtract one from the
        // target index to make the behavior more intuitive

        if (this.nodeDropTarget.position > 0
            && this.nodeDropTarget.data.name === this.draggedNode.data.parent.data.name
            && this.draggedNode.data.parent.data.child_names.indexOf(
              this.draggedNode.data.data.name) < this.nodeDropTarget.position)
        {
          this.nodeDropTarget.position -= 1;
        }

        // Also replace __forest_root with the empty string if present
        if (this.nodeDropTarget.data.name === "__forest_root")
        {
          this.nodeDropTarget.data.name = "";
        }

        // Three possible cases:

        // 1. Valid position, replace == false
        //    In this case we just call MoveNode with the given
        //    parent node and index
        if (this.nodeDropTarget.position >= 0
            && !this.nodeDropTarget.replace)
        {
          this.move_service.callService(
            new ROSLIB.ServiceRequest({
              node_name: this.draggedNode.data.data.name,
              new_parent_name: this.nodeDropTarget.data.name,
              new_child_index: this.nodeDropTarget.position
            }),
            function(response)
            {
              if (response.success)
              {
                console.log("Successfully moved node!");
              }
              else
              {
                this.props.onError("Failed to move node: " + response.error_message);
              }
            }.bind(this));
        }
        // 2. Valid position, replace == true
        //    First, remove the dropped node from its parent to prevent cycles.
        //    Then, move the node at the selected position to be our child,
        //    and finally move the dropped node to the indicated position as in 1.
        else if (this.nodeDropTarget.position >= 0
                 && this.nodeDropTarget.replace)
        {
          this.move_service.callService(
            new ROSLIB.ServiceRequest({
              node_name: this.draggedNode.data.data.name,
              new_parent_name: ''
            }),
            function(response)
            {
              if (response.success)
              {
                console.log("Successfully removed dropped node from its parent!");
                this.move_service.callService(
                  new ROSLIB.ServiceRequest({
                    // nodeDropTarget is the *parent* of the node we want to move!
                    node_name: this.nodeDropTarget.data.child_names[this.nodeDropTarget.position],
                    new_parent_name: this.draggedNode.data.data.name,
                    new_child_index: -1
                  }),
                  function(response)
                  {
                    if (response.success)
                    {
                      console.log("Successfully moved old node to be a child of dropped node, "
                                  + "now moving dropped node!");
                      this.move_service.callService(
                        new ROSLIB.ServiceRequest({
                          node_name: this.draggedNode.data.data.name,
                          // Now the parent of the node we moved first will become our parent!
                          new_parent_name: this.nodeDropTarget.data.name,
                          new_child_index: this.nodeDropTarget.position
                        }),
                        function(response)
                        {
                          if (response.success)
                          {
                            console.log("Successfully moved dropped node to the place "
                                        + "the old node was in!");
                          }
                          else
                          {
                            this.props.onError("Failed to move node: " + response.error_message);
                          }
                        }.bind(this));
                    }
                    else
                    {
                      this.props.onError("Failed to move node: " + response.error_message);
                    }
                  }.bind(this));
              }
              else
              {
                this.props.onError("Failed to move node: " + response.error_message);
              }
            }.bind(this));
        }
        // 3. position === -1, replace == true
        //    Call ReplaceNode with the given node name
        else if (this.nodeDropTarget.position == -1
                 && this.nodeDropTarget.replace)
        {
          this.replace_service.callService(
            new ROSLIB.ServiceRequest({
              old_node_name: this.nodeDropTarget.data.name,
              new_node_name: this.draggedNode.data.data.name
            }),
            function(response)
            {
              if (response.success)
              {
                console.log("Successfully replaced old node with dropped node!");
              }
              else
              {
                this.props.onError("Failed to replace node: " + response.error_message);
              }
            }.bind(this));
        }
        else
        {
          this.props.onError("Unexpected data for dropping a node :(");
        }
        d3.event.preventDefault();
        d3.event.stopPropagation();
      }
      else
      {
        console.log("should trigger the click event, hopefully?");
      }
    }

    this.dragging = false;

    d3.select(this.svg_ref.current).select("g.drop_targets")
      .attr("visibility", "hidden")
      .selectAll(".drop_target")
      .attr("visibility", "");

    d3.select(this.viewport_ref.current)
      .on("mousemove.drag_node", null)
      .on("mouseup.drag_node", null);
  }

  nodeClickHandler(d, index, group)
  {
    this.props.onSelectionChange(d.data.name);
  }

  nodeMousedownHandler(d, domObject)
  {
    if ((d3.event.buttons & 1) != 1)
    {
      return;
    }

    // No dragging the fake root!
    if (d.data.name == "__forest_root")
    {
      return;
    }

    this.draggedNode = {
      startCoords: d3.mouse(domObject),
      domObject: domObject,
      data: d
    };

    this.dragging = false;

    // Add move and mouseup handlers to viewport, and just to be sure,
    // remove the drag_io handlers in case they're present
    d3.select(this.viewport_ref.current)
      .on("mousemove.drag_io", null)
      .on("mouseup.drag_io", null)
      .on("mousemove.drag_node", this.canvasNodeDragMoveHandler.bind(this))
      .on("mouseup.drag_node", this.canvasNodeDragUpHandler.bind(this));

    d3.event.preventDefault();
    d3.event.stopPropagation();
  }

  dropTargetDefaultMouseoverHandler(domElement, datum)
  {
    this.nodeDropTarget = datum;
    d3.select(domElement).attr("opacity", 0.8);
  }

  dropTargetDefaultMouseoutHandler(domElement, datum)
  {
    this.nodeDropTarget = null;
    d3.select(domElement).attr("opacity", 0.2);
  }

  resetView()
  {
    var viewport = d3.select(this.viewport_ref.current);
    var height = viewport.node().getBoundingClientRect().height;

    var container = d3.select(this.svg_ref.current);

    viewport
      .call(this.zoomObject.scaleExtent([0.3, 1.0]).on("zoom", function () {
        container.attr("transform", d3.event.transform);
      }))
      .call(this.zoomObject.translateTo, 0.0, (height * 0.5) - 10.0);
  }
}

class NewNode extends Component
{
  constructor(props)
  {
    super(props);

    if (props.node) {
      this.state = {
        name: props.node.name,
        isValid: true,
        options: this.getDefaultValues(props.node.options),
        inputs: this.getDefaultValues(props.node.inputs,
                                      props.node.options),
        outputs: this.getDefaultValues(props.node.outputs,
                                       props.node.outputs)
      };
    }
    else
    {
      this.state = {
        name: '',
        isValid: false,
        options: [],
        inputs: [],
        outputs: []
      };
    }

    this.add_node_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'add_node',
      serviceType: 'ros_bt_py_msgs/AddNode'
    });

    this.selectRef = React.createRef();

    this.nameChangeHandler = this.nameChangeHandler.bind(this);
    this.updateValidity = this.updateValidity.bind(this);
    this.updateValue = this.updateValue.bind(this);
    this.onClickAdd = this.onClickAdd.bind(this);
  }

  render()
  {
    if (this.props.node === null)
    {
      return (
        <div className="d-flex flex-column">
          No Node Selected
        </div>
      );
    }

    return(
      <div className="d-flex flex-column">
        <button className="btn btn-block btn-primary"
                disabled={!this.state.isValid}
                onClick={this.onClickAdd}>Add to Tree</button>
        <label className="pt-2 pb-2">Parent
          <select className="custom-select d-block"
                  disabled={this.props.parents.length == 0}
                  ref={this.selectRef}
                  defaultValue={ (this.props.parents.length > 0) ? this.props.parents[0].name : null }>
            {
              this.props.parents
                .map(x => (<option key={x.name} value={x.name}>{x.name}</option>))
            }
          </select>
        </label>
        <EditableNode key={this.props.node.module
                           + this.props.node.node_class
                           + this.props.node.name}
                      name={this.state.name}
                      nodeClass={this.props.node.node_class}
                      updateValidity={this.updateValidity}
                      updateValue={this.updateValue}
                      nameChangeHandler={this.nameChangeHandler}
                      options={this.state.options}
                      inputs={this.state.inputs}
                      outputs={this.state.outputs}
        />
      </div>
    );
  }

  updateValidity(newValidity)
  {
    this.setState({isValid: newValidity || false});
  }

  onClickAdd()
  {
    var msg = this.buildNodeMessage();
    console.log('trying to add node to tree:');
    console.log(msg);
    this.add_node_service.callService(
      new ROSLIB.ServiceRequest({
        tree_name: '',
        parent_name: this.selectRef.current.value || '',
        node: msg,
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

  buildNodeMessage()
  {
    return {
      is_subtree: false,
      module: this.props.node.module,
      node_class: this.props.node.node_class,
      name: this.state.name,
      options: this.state.options.map(x => {
        var option = {
          key: x.key,
          serialized_value: ''
        };
        if (x.value.type === 'type') {
          if (python_builtin_types.indexOf(x.value.value) >= 0)
          {
            x.value.value = '__builtin__.' + x.value.value;
          }
          option.serialized_value = JSON.stringify({
            "py/type": x.value.value
          });
        }
        else if (x.value.type.startsWith('__'))
        {
          x.value.value["py/object"] = x.value.type.substring('__'.length);
          option.serialized_value = JSON.stringify(x.value.value);
        }
        else
        {
          option.serialized_value = JSON.stringify(x.value.value);
        }
        return option;
      }),
      child_names: []
    };
  }

  nameChangeHandler(event)
  {
    this.setState({name: event.target.value});
  }

  updateValue(paramType, key, new_value)
  {
    var map_fun = function(x)
    {
      if (x.key === key) {
        return {
          key: key,
          value: {
            type: x.value.type,
            value: new_value
          }
        };
      }
      else
      {
        return x;
      }
    };

    if (paramType.toLowerCase() === 'options')
    {
      var ref_keys = this.props.node.options
          .filter(x => prettyprint_type(x.serialized_value).startsWith('OptionRef('))
          .map(x => [x.key, prettyprint_type(x.serialized_value).substring(
            'OptionRef('.length, prettyprint_type(x.serialized_value).length - 1)])
          .filter(x => x[1] === key);
      this.setState(
        (prevState, props) =>
          {
            var new_options = prevState.options.map(map_fun);
            // update the options in our state that are references to
            // the changed key - this will discard any values entered
            // already, but they'd be incompatible anyway

            var resolved_options = new_options.map(x => {
              var refData = ref_keys.find(ref => ref[0] === x.key);
              if (refData)
              {
                var optionType = new_options.find(opt => opt.key === refData[1]);
                if (optionType)
                {
                  return {
                    key: x.key,
                    value: getDefaultValue(optionType.value.value.replace('__builtin__.', ''))
                  };
                }
              }
              return x;
            });
            return {options: resolved_options};
          });
    }
    else if (paramType.toLowerCase() === 'inputs')
    {
      this.setState(
        (prevState, props) =>
          {
            return {inputs: prevState.inputs.map(map_fun)};
          });
    }
    else if (paramType.toLowerCase() === 'outputs')
    {
      this.setState(
        (prevState, props) =>
          {
            return {outputs: prevState.outputs.map(map_fun)};
          });
    }
  }

  getDefaultValues(paramList, options)
  {
    options = options || [];

    return paramList.map(x => {
      return {
        key: x.key,
        value: getDefaultValue(
          prettyprint_type(x.serialized_value), options),
      };
    });
  }
}

class SelectedNode extends Component
{
  constructor(props)
  {
    super(props);

    var getValues = x =>
      {
        var type = prettyprint_type(x.serialized_type);
        var json_value = JSON.parse(x.serialized_value);
        if (type === 'type')
        {
          json_value = json_value['py/type']
            .replace('__builtin__.', '');
        }
        return {
          key: x.key,
          value: {type: type
                  .replace(/^basestring$/, 'string')
                  .replace(/^str$/, 'string')
                  .replace(/^unicode$/, 'string'),
                  value: json_value}
        };
      };
    if (props.node) {
      this.state = {
        name: props.node.name,
        isValid: true,
        options: props.node.options.map(getValues),
        inputs: props.node.inputs.map(getValues),
        outputs: props.node.outputs.map(getValues)
      };
    }
    else
    {
      this.state = {
        name: '',
        isValid: false,
        options: [],
        inputs: [],
        outputs: []
      };
    }

    this.nameChangeHandler = this.nameChangeHandler.bind(this);
    this.updateValidity = this.updateValidity.bind(this);
    this.updateValue = this.updateValue.bind(this);
    this.onClickDelete = this.onClickDelete.bind(this);
    this.onClickUpdate = this.onClickUpdate.bind(this);
  }

  componentDidMount()
  {
    this.set_options_service = new ROSLIB.Service({
      ros: this.props.ros,
      name: this.props.bt_namespace + 'set_options',
      serviceType: 'ros_bt_py_msgs/SetOptions'
    });

    this.remove_node_service = new ROSLIB.Service({
      ros: this.props.ros,
      name: this.props.bt_namespace + 'remove_node',
      serviceType: 'ros_bt_py_msgs/RemoveNode'
    });
  }

  nameChangeHandler(event)
  {
    this.setState({name: event.target.value});
  }

  onClickDelete(event)
  {
    if (!window.confirm("Really delete node " + this.props.node.name + "?"))
    {
      // Do nothing if user doesn't confirm
      return;
    }
    this.remove_node_service.callService(
      new ROSLIB.ServiceRequest({
        node_name: this.props.node.name
      }),
      function(response) {
        if (response.success) {
          console.log('Removed node!');
        }
        else {
          this.props.onError('Failed to remove node '
                             + this.props.node.name + ': '
                             + response.error_message);
        }
      }.bind(this));
  }

  onClickUpdate(event)
  {
    console.log('updating node');
    this.set_options_service.callService(
      new ROSLIB.ServiceRequest({
        tree_name: '',
        node_name: this.props.node.name,
        rename_node: true,
        new_name: this.state.name,
        options: this.state.options.map(x => {
          var option = {
            key: x.key,
            serialized_value: ''
          };
          if (x.value.type === 'type') {
            if (python_builtin_types.indexOf(x.value.value) >= 0)
            {
              x.value.value = '__builtin__.' + x.value.value;
            }
            option.serialized_value = JSON.stringify({
              "py/type": x.value.value
            });
          }
          else if (x.value.type.startsWith('__'))
          {
            x.value.value["py/object"] = x.value.type.substring('__'.length);
            option.serialized_value = JSON.stringify(x.value.value);
          }
          else
          {
            option.serialized_value = JSON.stringify(x.value.value);
          }
          return option;
        })
      }),
      function(response) {
        if (response.success) {
          console.log('Updated node!');
        }
        else {
          this.props.onError('Failed to update node '
                             + this.props.node.name + ': '
                             + response.error_message);
        }
      }.bind(this));
  }

  render()
  {
    return (
      <div className="d-flex flex-column">
        <div className="btn-group d-flex mb-2" role="group">
          <button className="btn btn-primary w-100"
                  disabled={!this.state.isValid}
                  onClick={this.onClickUpdate}>Update Node</button>
          <button className="btn btn-danger w-100"
                  onClick={this.onClickDelete}>Delete Node</button>
        </div>
        <EditableNode key={this.props.node.module
                           + this.props.node.node_class
                           + this.props.node.name}
                      name={this.state.name}
                      nodeClass={this.props.node.node_class}
                      updateValidity={this.updateValidity}
                      updateValue={this.updateValue}
                      nameChangeHandler={this.nameChangeHandler}
                      options={this.state.options}
                      inputs={this.state.inputs}
                      outputs={this.state.outputs}
        />
      </div>
    );
  }

  updateValidity(newValidity)
  {
    this.setState({isValid: newValidity || false});
  }

  updateValue(paramType, key, new_value)
  {
    var map_fun = function(x)
    {
      if (x.key === key) {
        return {
          key: key,
          value: {
            type: x.value.type,
            value: new_value
          }
        };
      }
      else
      {
        return x;
      }
    };

    if (paramType.toLowerCase() === 'options')
    {
      // All of these are lists containing lists of [key, ref_key]
      //
      // That is, if options = { foo : int, bar : OptionRef(foo) }
      // ref_keys will be [[bar, foo]]
      var ref_keys = this.props.nodeInfo.options
          .filter(x => prettyprint_type(x.serialized_value).startsWith('OptionRef('))
          .map(x => [x.key, prettyprint_type(x.serialized_value).substring(
            'OptionRef('.length, prettyprint_type(x.serialized_value).length - 1)])
          .filter(x => x[1] === key);
      var input_option_ref_keys = this.props.nodeInfo.inputs
          .filter(x => prettyprint_type(x.serialized_value).startsWith('OptionRef('))
          .map(x => [x.key, prettyprint_type(x.serialized_value).substring(
            'OptionRef('.length, prettyprint_type(x.serialized_value).length - 1)])
          .filter(x => x[1] === key);
      var output_option_ref_keys = this.props.nodeInfo.inputs
          .filter(x => prettyprint_type(x.serialized_value).startsWith('OptionRef('))
          .map(x => [x.key, prettyprint_type(x.serialized_value).substring(
            'OptionRef('.length, prettyprint_type(x.serialized_value).length - 1)])
          .filter(x => x[1] === key);
      this.setState(
        (prevState, props) =>
          {
            // Replace the option value
            var new_options = prevState.options.map(map_fun);

            // update the options in our state that are references to
            // the changed key - this will discard any values entered
            // already, but they'd be incompatible anyway

            var resolve_refs = current_item => {
              // See if the current option references the changed key
              var refData = ref_keys.find(ref => ref[0] === current_item.key);
              if (refData)
              {
                // If it does, find the type of the referred key
                var optionType = new_options.find(opt => opt.key === refData[1]);
                if (optionType)
                {
                  // Get a default value for the type indicated by the
                  // referenced option
                  return {
                    key: current_item.key,
                    value: getDefaultValue(optionType.value.value.replace('__builtin__.', ''))
                  };
                }
              }
              return current_item;
            };

            var resolved_options = new_options.map(resolve_refs);
            var newState = {options: resolved_options};
            // if there's inputs or outputs that reference the changed
            // option key, update them too (this is just for display
            // and won't change anything in the backend)
            if (input_option_ref_keys.length > 0)
            {
              newState.inputs = prevState.inputs.map(resolve_refs);
            }
            if (output_option_ref_keys.length > 0)
            {
              newState.outputs = prevState.outputs.map(resolve_refs);
            }
            return newState;
          });
    }
    else if (paramType.toLowerCase() === 'inputs')
    {
      this.setState(
        (prevState, props) =>
          {
            return {inputs: prevState.inputs.map(map_fun)};
          });
    }
    else if (paramType.toLowerCase() === 'outputs')
    {
      this.setState(
        (prevState, props) =>
          {
            return {outputs: prevState.outputs.map(map_fun)};
          });
    }
  }
}

class EditableNode extends Component
{
  constructor(props)
  {
    super(props);
  }

  render()
  {
    var compareKeys = function(a, b) {
      if (a.key === b.key) {
        return 0;
      }
      if (a.key < b.key) {
        return -1;
      }
      return 1;
    };
    return(
      <div className="d-flex flex-column">
        <input className="form-control-lg mb-2"
               type="text"
               value={this.props.name}
               onChange={this.props.nameChangeHandler}/>
        <h4 className="text-muted">{this.props.nodeClass}</h4>
        {this.renderParamInputs(this.props.options.sort(compareKeys), 'options')}
        {this.renderParamDisplays(this.props.inputs.sort(compareKeys), 'inputs')}
        {this.renderParamDisplays(this.props.outputs.sort(compareKeys), 'outputs')}
      </div>
    );
  }

  updateValue(paramType, key, new_value)
  {
    var map_fun = function(x)
    {
      if (x.key === key) {
        return {
          key: key,
          value: {
            type: x.value.type,
            value: new_value
          }
        };
      }
      else
      {
        return x;
      }
    };

    if (paramType.toLowerCase() === 'options')
    {
      var ref_keys = this.props.options
          .filter(x => prettyprint_type(x.serialized_value).startsWith('OptionRef('))
          .map(x => [x.key, prettyprint_type(x.serialized_value).substring(
            'OptionRef('.length, prettyprint_type(x.serialized_value).length - 1)])
          .filter(x => x[1] === key);

      var new_options = this.props.options.map(map_fun);
      // update the options in our state that are references to
      // the changed key - this will discard any values entered
      // already, but they'd be incompatible anyway

      var resolved_options = new_options.map(x => {
        var refData = ref_keys.find(ref => ref[0] === x.key);
        if (refData)
        {
          var optionType = new_options.find(opt => opt.key === refData[1]);
          if (optionType)
          {
            return {
              key: x.key,
              value: getDefaultValue(optionType.value.value.replace('__builtin__.', ''))
            };
          }
        }
        return x;
      });

      this.props.setOptions(resolved_options);
    }
    else if (paramType.toLowerCase() === 'inputs')
    {
      this.props.setInputs(this.props.inputs.map(map_fun));
    }
    else if (paramType.toLowerCase() === 'outputs')
    {
      this.props.setOutputs(this.props.outputs.map(map_fun));
    }
  }

  inputForValue(paramItem, onValidityChange, onNewValue)
  {
    var valueType = paramItem.value.type;
    var changeHandler = (event) => {};

    if (valueType === 'int')
    {
      // Number input with integer increments
      changeHandler = (event) =>
          {
            var newValue = Math.round(event.target.value);
            if (isNaN(newValue))
            {
              newValue = 0;
            }
            onNewValue(newValue);
          };

      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}
            <input type="number" name="integer"
                   className="form-control"
                   onChange={changeHandler}
                   placeholder="integer"
                   step="1.0"
                   value={paramItem.value.value}>
            </input>
          </label>
        </div>
      );
    }
    if (valueType === 'float')
    {
      // Number input with free increments
      changeHandler = (event) =>
          {
            var newValue = parseFloat(event.target.value);
            if (isNaN(newValue))
            {
              newValue = 0;
            }
            onNewValue(newValue);
          };

      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}
            <input type="number" name="float"
                   step="any"
                   className="form-control"
                   onChange={changeHandler}
                   placeholder="float"
                   value={paramItem.value.value}>
            </input>
          </label>
        </div>
      );
    }
    else if (valueType === 'string')
    {
      // Regular input
      changeHandler = (event) =>
        {
          onNewValue(event.target.value || '');
        };

      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}
            <input type="text"
                   className="form-control mt-2"
                   value={paramItem.value.value}
                   onChange={changeHandler}/>
          </label>
        </div>
      );
    }
    else if (valueType === 'type')
    {
      // Regular input
      changeHandler = (event) =>
        {
          var newTypeName = event.target.value || '';
          if (python_builtin_types.indexOf(newTypeName) >= 0)
          {
            onNewValue('__builtin__.' + newTypeName);
          }
          else
          {
            onNewValue(newTypeName);
          }
        };

      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}
            <input type="text"
                   className="form-control mt-2"
                   value={paramItem.value.value}
                   onChange={changeHandler}/>
          </label>
        </div>
      );
    }
    else if (valueType === 'bool')
    {
      // Checkbox
      changeHandler = (event) =>
        {
          onNewValue(event.target.checked || false);
        };

      var checkID = 'input_checkbox_' + uuid();
      return (
        <div className="custom-control custom-checkbox m-1">
          <input type="checkbox"
                 id={checkID}
                 className="custom-control-input"
                 checked={paramItem.value.value}
                 onChange={changeHandler} />
          <label className="custom-control-label d-block"
                 htmlFor={checkID}>
            {paramItem.key}
        </label>

        </div>
      );
    }
    else if (valueType === 'unset_optionref')
    {
      return (
        <div className="form-group m-1">
          <label className="d-block">{paramItem.key}
            <input type="text"
                   className="form-control mt-2"
                   value={paramItem.value.value}
                   disabled={true}/>
          </label>
        </div>
      );
    }
    // TODO(nberg): implement these two

    // else if (valueType === 'list')
    // {

    // }
    // else if (valueType === 'dict')
    // {

    // }
    else  // if (valueType === 'object')
    {
      // textarea with JSON.stringify(value), parse back when changed
      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}
            <JSONInput initialValue={JSON.stringify(paramItem.value.value)}
                       onValidityChange={onValidityChange}
                       onNewValue={onNewValue}/>
          </label>
        </div>
      );
    }
  }

  displayForValue(paramItem)
  {
    var valueType = paramItem.value.type;

    if (valueType === 'int'
        || valueType === 'float'
        || valueType ==='string')
    {
      // Number input with integer increments
      return (
        <Fragment>
          <h5>{paramItem.key}</h5>
          <span>{paramItem.value.value}</span>
        </Fragment>
      );
    }
    else if (valueType === 'type')
    {
      return (
        <Fragment>
          <h5>{paramItem.key}</h5>
          <pre>{paramItem.value.value}</pre>
        </Fragment>
      );
    }
    else if (valueType === 'boolean')
    {
      return (
        <Fragment>
          <h5>{paramItem.key}</h5>
          <pre>{paramItem.value.value ? 'True' : 'False'}</pre>
        </Fragment>
      );
    }
    else if (valueType === 'unset_optionref')
    {
      return (
        <Fragment>
          <h5>{paramItem.key}</h5>
          <pre className="text-muted">{paramItem.value.value}</pre>
        </Fragment>
      );
    }
    // TODO(nberg): implement these two

    // else if (valueType === 'list')
    // {

    // }
    // else if (valueType === 'dict')
    // {

    // }
    else  // if (valueType === 'object')
    {
      console.log('item with non-basic type: ', paramItem);
      return (
        <Fragment>
          <h5>{paramItem.key}</h5>
          <pre>{JSON.stringify(paramItem.value.value, null, 2)}</pre>
        </Fragment>
      );
    }
  }

  renderParamInputs(params, name)
  {
    var param_rows = params.map(x => {
      return (
        <div className="list-group-item"
             key={name + x.key}>
            {
              this.inputForValue(
                x,
                this.props.updateValidity,
                (newVal) => this.props.updateValue(name, x.key, newVal))
            }
        </div>
      );
    });

    return (
      <div className="mb-2">
        <h5>{name}</h5>
        <div className="list-group">
          {param_rows}
        </div>
      </div>
    );
  }

  renderParamDisplays(params, name)
  {
    var param_rows = params.map(x => {
      return (
        <div className="list-group-item"
             key={name + x.key}>
          {this.displayForValue(x)}
        </div>
      );
    });

    return (
      <div className="mb-2">
        <h5>{name}</h5>
        <div className="list-group">
            {param_rows}
        </div>
      </div>
    );
  }

  getDefaultValues(paramList, options)
  {
    options = options || [];

    return paramList.map(x => {
      return {
        key: x.key,
        value: getDefaultValue(
          prettyprint_type(x.serialized_value), options),
      };
    });
  }
}

class BehaviorTreeEdge extends Component
{
  constructor(props)
  {
    super(props);

    this.onClickDelete = this.onClickDelete.bind(this);
  }

  componentDidMount()
  {
    this.unwireClient = new ROSLIB.Service({
      ros: this.props.ros,
      name: this.props.bt_namespace + 'unwire_data',
      serviceType: 'ros_bt_py_msgs/WireNodeData'
    });
  }

  onClickDelete()
  {
    console.log(this.props.edge);
    this.unwireClient.callService(
      new ROSLIB.ServiceRequest({
        wirings: [{
          source: this.props.edge.source,
          target: this.props.edge.target
        }]
      }),
      function(response) {
        if (!response.success) {
          this.props.onError(response.error_message);
        }
      });
  }

  render()
  {
    return(
      <div className="d-flex flex-column">
        <div className="btn-group d-flex mb-2" role="group">
          <button className="btn btn-danger w-100"
                  onClick={this.onClickDelete}>Delete Edge</button>
        </div>
        <div className="row">
          <div className="col">
            <a href="#"
               className="text-primary"
               onClick={()=>this.props.onSelectionChange(this.props.edge.source.node_name)}>
              {this.props.edge.source.node_name}.
            </a>
            <span>{this.props.edge.source.data_kind}.</span>
            <span>{this.props.edge.source.data_key}</span>
          </div>
          <div className="col">
            <span aria-hidden="true" className="fas fa-lg fa-long-arrow-alt-right" />
            <span className="sr-only">is connected to</span>
          </div>
           <div className="col">
             <a href="#"
                className="text-primary"
                onClick={()=>this.props.onSelectionChange(this.props.edge.target.node_name)}>
              {this.props.edge.target.node_name}.
            </a>
            <span>{this.props.edge.target.data_kind}.</span>
            <span>{this.props.edge.target.data_key}</span>
          </div>
        </div>
      </div>
    );
  }
}



class JSONInput extends Component
{
  constructor(props)
  {
    super(props);

    var is_valid = false;

    try {
      JSON.parse(props.initialValue);
      is_valid = true;
    }
    catch (e)
    {
      is_valid = false;
    }

    this.state = {
      value: props.initialValue,
      is_valid: is_valid
    };

    this.changeHandler = this.changeHandler.bind(this);
  }

  changeHandler(event)
  {
    try
    {
      console.log('parsed:');
      console.log(JSON.parse(event.target.value));
    }
    catch (e)
    {
    }
    this.setState({value: event.target.value});
    try
    {
      this.props.onNewValue(JSON.parse(event.target.value));
      this.setState({is_valid: true});
      this.props.onValidityChange(true);
    }
    catch(e)
    {
      // Do nothing
      this.setState({is_valid: false});
      this.props.onValidityChange(false);
    }
  }

  render()
  {
    return (
      <input type="textarea"
             className={'form-control mt-2 ' + (this.state.is_valid ? '' : 'is-invalid')}
             value={this.state.value}
             onChange={this.changeHandler}/>
    );
  }
}

render(<App />, document.body);
