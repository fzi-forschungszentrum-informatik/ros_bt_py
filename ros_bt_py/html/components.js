
// import { Component } from 'preact';
// import { render } from 'preact';
// import { createRef } from 'preact';
var Component = React.Component;
var render = ReactDOM.render;
var createRef = React.createRef;
var Fragment = React.Fragment; //'x-fragment';

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
    return json_type['py/type'].replace('__builtin__.', '');
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
  else if (typeName === 'str' || typeName === 'basestring' || typeName === 'unicode')
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
    return {type: 'boolean',
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
    var optionType = options.find(x => {
      return x === typeName.substring(
        'OptionRef('.length, typeName.length - 1);
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
        value: 'None'
      };
    }
  }
  else
  {
    return {type: '__' + typeName,
            value: ''};
  }
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
          <td className="io_type">
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
        <h5 className="node_module">{this.props.node.module}</h5>
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
      available_nodes: [],
      package_name: 'ros_bt_py.nodes.sequence'
    };

    this.onError = props.onError;
    this.get_nodes_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'get_available_nodes',
      serviceType: 'ros_bt_py_msgs/GetAvailableNodes'
    });

    this.getNodes = this.getNodes.bind(this);
    this.handleChange = this.handleChange.bind(this);
  }

  componentDidMount()
  {
    this.getNodes('');
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

  handleChange(e)
  {
    this.setState({package_name: e.target.value});
  }

  render()
  {
    var items = this.state.available_nodes.map( (node) => {
      return (<NodeListItem node={node}
              key={node.module + node.node_class}
              onSelectionChange={this.props.onSelectionChange}/>);
    });
    return(
      <div className="available-nodes m-1">
        <div className="form-group">
          <button id="refresh"
                  className="btn btn-block btn-primary mt-2"
                  onClick={this.getNodes.bind(this, '')}>
            Refresh
          </button>
          <input type="text" id="package_name"
                 className="form-control mt-2"
                 value={this.state.package_name}
                 onChange={this.handleChange}/>
          <button id="load_package"
                  className="btn btn-block btn-primary mt-2"
                  onClick={this.getNodes.bind(this, this.state.package_name)}>
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

    this.state = {
      bt_namespace: '/tree_node/',
      ros_uri: 'ws://10.211.55.3:9090',
      selected_node: null,
      showDataGraph: true,
      last_tree_msg: null
    };
    this.ros = new ROSLIB.Ros({
      url : this.state.ros_uri
    });

    this.tree_topic = new ROSLIB.Topic({
      ros : this.ros,
      name : this.state.bt_namespace + 'tree',
      messageType : 'ros_bt_py_msgs/Tree'
    });

    // Bind these here so this works as expected in callbacks
    this.onError = this.onError.bind(this);
    this.onSelectionChange = this.onSelectionChange.bind(this);
    this.onTreeUpdate = this.onTreeUpdate.bind(this);
    this.findPossibleParents = this.findPossibleParents.bind(this);
  }

  onTreeUpdate(msg)
  {
    this.setState({last_tree_msg: msg});
  }

  findPossibleParents()
  {
    if (this.state.last_tree_msg)
    {
      return this.state.last_tree_msg.nodes
        .filter(node => (node.max_children < 0 || node.child_names.length < node.max_children));
    }
    return [];
  }

  componentDidMount()
  {
    this.tree_topic.subscribe(this.onTreeUpdate);
  }

  componentWillUnmount()
  {
    this.tree_topic.unsubscribe(this.onTreeUpdate);
  }

  onError(error_message)
  {
    console.log(error_message);
  }

  onSelectionChange(new_selected_node)
  {
    this.setState({selected_node: new_selected_node});
  }

  render()
  {
    return (
      <Fragment>
        <ExecutionBar ros={this.ros}
                      bt_namespace={this.state.bt_namespace}
                      onError={this.onError}/>

        <div className="container-fluid">
          <div className="row row-height">
            <div className="col scroll-col" id="nodelist_container">
              <NodeList ros={this.ros}
                        bt_namespace={this.state.bt_namespace}
                        onError={this.onError}
                        onSelectionChange={this.onSelectionChange}/>
            </div>
            <div className="col-9 scroll-col" id="main_pane">
              <div className="container-fluid">
                <div className="row edit_canvas">
                  <div className="col">
                    <D3BehaviorTreeEditor ros={this.ros}
                                          bt_namespace={this.state.bt_namespace}
                                          onSelectionChange={this.onSelectionChange}
                                          showDataGraph={this.state.showDataGraph}
                                          onError={this.onError}/>
                  </div>
                </div>
                <div className="row">
                  <button className="btn btn-primary btn-block mb-2"
                          onClick={function() {
                            this.setState(
                              (prevstate, props) => ({showDataGraph: !prevstate.showDataGraph})
                            );
                          }.bind(this)
                    }>
                    Toggle Data Graph
                  </button>
                </div>
                <div className="row">
                  <div className="col">
                    <SelectedNode
                      ros={this.ros}
                      bt_namespace={this.state.bt_namespace}
                      key={this.state.selected_node ? (this.state.selected_node.module + this.state.selected_node.class_name) : ''}
                      node={this.state.selected_node}
                      parents={this.findPossibleParents()}
                      />
                  </div>
                  <div className="col">
                    <RemoveNode ros={this.ros}
                                bt_namespace={this.state.bt_namespace}
                                onError={this.onError}/>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </Fragment>
    );
  }
}

function ExecutionBar(props)
{
  return (
    <header id="header" className="d-flex flex-column flex-md-row align-items-center control-bar">
      <DebugControls
        ros={props.ros}
        bt_namespace={props.bt_namespace}
        onError={props.onError}/>
      <TickControls
        ros={props.ros}
        bt_namespace={props.bt_namespace}
        onError={props.onError}/>
    </header>
  );
}

class TickControls extends Component
{
  constructor(props)
  {
    super(props);

    this.onError = props.onError;

    this.tick_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'control_tree_execution',
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
          this.onError(response.error_message);
        }
      });
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

    this.state = {value: false};

    this.onError = props.onError;

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

    this.onNewDebugSettings = this.onNewDebugSettings.bind(this);
    this.onClickStep = this.onClickStep.bind(this);
    this.handleChange = this.handleChange.bind(this);
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
    this.setState({value: msg.single_step});
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
          this.onError(response.error_message);
        }
      });
  }

  handleChange(event)
  {
    var enable = event.target.checked;
      this.set_execution_mode_service.callService(
    new ROSLIB.ServiceRequest({
      single_step: enable,
      collect_performance_data: true
    }),
    function(response) {
      if (enable) {
        console.log('enabled stepping');
      }
      else {
        console.log('disabled stepping');
      }
    });
    this.setState({value: event.target.value});
  }

  render()
  {
    return (
      <Fragment>
        <div className="form-check m-1">
          <input type="checkbox"
                 id="debugging"
                 className="form-check-input"
                 checked={this.state.value}
                 onChange={this.handleChange} />
          <label className="form-check-label"
                 htmlFor="debugging">Debug</label>
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

    this.state = {
      editable: true
    };

    this.horizontal_spacing = 80;
    this.vertical_spacing = 25;

    this.io_gripper_spacing = 10;
    this.max_io_gripper_size = 15;

    this.tree_topic = new ROSLIB.Topic({
      ros : props.ros,
      name : props.bt_namespace + 'tree',
      messageType : 'ros_bt_py_msgs/Tree'
    });

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

    this.svg_ref = createRef();
    this.viewport_ref = createRef();

    this.onSelectionChange = props.onSelectionChange;
    this.onError = props.onError;

    this.onTreeUpdate = this.onTreeUpdate.bind(this);
    this.getIOCoords = this.getIOCoords.bind(this);
    this.getIOCoordsFromNode = this.getIOCoordsFromNode.bind(this);
  }

  componentDidMount()
  {
    // Give Viewport pan / zoom
    var viewport = d3.select(this.viewport_ref.current);
    var width = viewport.node().getBoundingClientRect().width;

    var tmpzoom = d3.zoom();
    var container = d3.select(this.svg_ref.current);

    viewport
      .call(tmpzoom.scaleExtent([0.3, 1.0]).on("zoom", function () {
        container.attr("transform", d3.event.transform);
      }))
      .call(tmpzoom.translateBy, width * 0.5, 10.0);

    this.tree_topic.subscribe(this.onTreeUpdate);
  }

  componentWillUnmount()
  {
    this.tree_topic.unsubscribe(this.onTreeUpdate);
  }

  render()
  {
    return (
      <svg id="editor_viewport"
           ref={this.viewport_ref}
           className="reactive-svg">
        <g id="container" ref={this.svg_ref}>
          // order is important here - SVG draws things in the order
          // they appear in the markup!
          <g className="edges"/>
          <g className="vertices"/>
          // Data Graph should be above the node graph
          // (since it can be toggled on and off)
          <g className="data_graph">
            // We want the edges below the vertices here because that looks nicer
            <g className="data_edges" />
            <g className="data_vertices" />
          </g>
          // This is for the targets that appear when the user is dragging
          // a node to reposition it.
          // Obviously, these should be above anything else!
          <g className="drop_targets" />
        </g>
      </svg>
    );
  }

  componentDidUpdate()
  {

    // Disable all interaction (except for zooming and panning) when
    // the tree isn't editable
    if (this.state.editable)
    {

    }

    // Hide or show data graph
    if (this.props.showDataGraph)
    {
      d3.select(this.svg_ref.current).select(".data_graph").attr("visibility", "hidden");
    }
    else
    {
      d3.select(this.svg_ref.current).select(".data_graph").attr("visibility", "visible");
    }

  }

  onTreeUpdate(tree_msg)
  {
    this.drawEverything(tree_msg);

    this.setState({editable: tree_msg.state === "EDITABLE"});
  }

  drawEverything(tree_msg)
  {
    var forest_root = {
      name: "__forest_root",
      child_names: [],
      inputs: [],
      outputs: [],
      options: []
    };

    tree_msg.nodes.push(forest_root);
    // Update the visual tree
    var parents = {};
    var node_dict = {};
    // Find parents for all nodes once
    (function(){
      for (var i in tree_msg.nodes) {
        var node = tree_msg.nodes[i];
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
        })(tree_msg.nodes);

    root.sort(function(a, b) {
      if (a.depth !== b.depth) {
        return b.depth - a.depth;
      }
      if (a.parent !== b.parent) {
        a = a.parent;
        b = b.parent;
        while (a_parent !== b_parent) {
          a = a.parent;
          b = b.parent;
        }
        console.log("shouldn't happen");
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
        .data(root.descendants(), function(node) {
          return node.id;
        });

    node.exit().remove();

    var nodeEnter = node.enter();

    this.drawNodes(nodeEnter);

    // node = nodeEnter.merge(node);
    node = g_vertex
      .selectAll(".btnode")
      .data(root.descendants(), function(node) {
        return node.id;
      });

    this.updateNodes(node);

    // k is the zoom level - we need to apply this to the values we get
    // from getBoundingClientRect, or we get fun scaling effects.
    var zoom = d3.zoomTransform(d3.select(this.viewport_ref.current).node()).k;

    // Find the maximum size of all the nodes, for layout purposes
    var max_size = [0,0];
    g_vertex.selectAll('.btnode').data(root.descendants(), d => d.id)
      .each(function(d, index){
        var rect = this.getBoundingClientRect();
        rect.x /= zoom;
        rect.y /= zoom;
        rect.width /= zoom;
        rect.height /= zoom;
        d._size = rect;
        this.parentElement.setAttribute('width', rect.width);
        this.parentElement.setAttribute('height', rect.height);
      });

    var tree_size = [width - max_size[0], height - (40 + max_size[1])];

    var tree = d3.flextree()
        .nodeSize(function(node) {
          return [node._size.width + this.horizontal_spacing,
                  node._size.height + this.vertical_spacing];
        }.bind(this))
    (root);

    // Move new nodes to their starting positions
    g_vertex.selectAll(".node").data(root.descendants(), d => d.id)
      .filter(d => d._entering)
      .attr("transform", function(d) {
        // Start at parent position
        var p = this.findExistingParent(d);
        return "translate(" + Math.round(p.x) + "," + Math.round(p.y) + ") scale(0.1)";
      }.bind(this));

    var link = g_edge.selectAll(".link")
        .data(tree.links(), function(d) { return '' + d.source.id + d.target.id; });
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
    node = g_vertex.selectAll(".node")
      .data(root.descendants(), function(node) {return node.id;});

    var t = d3.transition()
        .duration(250);
    node.transition(t)
      .attr("transform", function(d) {
        // animate to actual position
        return "translate(" + Math.round(d.x - d._size.width / 2.0) + "," + Math.round(d.y) + ") scale(1.0)";
      });

    this.drawDataGraph(g_data, node.data(), tree_msg.data_wirings);

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
    // console.log(root);
  }

  drawNodes(selection)
  {
    selection.each(function(d) {
      d._entering = true;
      d._show = true;
      d.x = 0;
      d.y = 0;
    });

    var fo = selection.append('foreignObject')
        .attr("class", function(d) {
          return "node" + (d.children ? " node--internal" : " node--leaf");
        })
        .on("click", this.nodeClickHandler.bind(this))
        .on("mousedown", this.nodeMousedownHandler.bind(this));

    var div = fo
        .append("xhtml:body")
        .attr("class", "btnode");
  }

  updateNodes(selection)
  {
    // Update name
    var title = selection.selectAll(".node_name").data(function(d) {
      return [d];
    });
    title = title.enter().append("h3").attr("class", "node_name").merge(title);
    title.html(function(d) {
      return d.id;
    });

    var container = selection.selectAll(".table_container").data(d => [d]);
    container = container.enter()
      .append("div")
      .attr("class", "table_container")
      .merge(container);

    var tables = container.selectAll("table").data(function(d) {
      return d3.entries({
        inputs: d.data.inputs || [],
        outputs: d.data.outputs || [],
        options: d.data.options || []
      }).filter(x => x.value.length > 0);
    }, d => d.key);
    tables = tables.enter().append("table").attr("class", d => [d.key]).merge(tables);

    tables.selectAll("thead").data(d=>[d]).enter()
      .append("thead")
      .append("tr")
      .append("th")
      .text(d => d.key);
    tables.selectAll("tbody").data(d=>[d]).enter()
      .append("tbody")
      .attr("class", d => d.key);

    this.fillTables(tables.select("tbody"));
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

  findExistingParent(d)
  {
    while (d._entering && d.parent) {
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

  drawDataGraph(g_data, data, wirings)
  {
    // Add the edge container first so the vertices draw over the
    // edges, which looks nicer
    var edges = g_data.selectAll("g.data_edges");
    var vertices = g_data.selectAll("g.data_vertices");

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
      }.bind(this)),
      vertices);
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

    var gripperSize = this.getGripperSize(node._size.height,
                                          inputs.length,
                                          outputs.length);
    if (data_kind === 'inputs')
    {
      coords = this.getGripperCoords(
        node.data.inputs.findIndex(x => x.key === data_key) || 0,
        /*right=*/false,
        gripperSize,
        node._size.width);
    }
    else if (data_kind === 'outputs')
    {
      coords = this.getGripperCoords(
        node.data.outputs.findIndex(x => x.key === data_key) || 0,
        /*right=*/true,
        gripperSize,
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
      coords.x += gripperSize * 0.5;
      coords.y += gripperSize * 0.5;
    }
    return {
      x: node.x + coords.x,
      y: node.y + coords.y,
      gripperSize: gripperSize
    };
  }

  drawDataEdges(edge_selection, edge_data, vertex_selection)
  {
    var link = edge_selection.selectAll(".data-link").data(edge_data);
    link.exit().remove();

    link = link
      .enter()
      .append("path")
      .attr("class", "data-link")
      .attr("d", function(d) {
        var lineGen = d3.line()
            .x(d => d.x)
            .y(d => d.y)
            .curve(d3.curveCatmullRom.alpha(0.9));
        return lineGen(d.points);
      })
      .on("mouseover", this.DataEdgeDefaultMouseoverHandler)
      .on("mouseout", this.DataEdgeDefaultMouseoutHandler)
      .merge(link);
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
      .on("mouseover", this.IOGroupDefaultMouseoverHandler)
      .on("mouseout", this.IOGroupDefaultMouseoutHandler)
      .merge(groups);
    groups
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
    var my_data = d3.select(this).datum();
    d3.select(this).classed("data-hover", true);

    // select source gripper
    selectIOGripper(vertex_selection, my_data.source)
      .each(function(d) {
        this.dispatchEvent(new CustomEvent("mouseover"));
      });

    // select target gripper
    selectIOGripper(vertex_selection, my_data.target)
      .each(function(d) {
        this.dispatchEvent(new CustomEvent("mouseover"));
      });
  }

  DataEdgeDefaultMouseoutHandler(d, index, group)
  {
    var vertex_selection = d3.select(this.parentNode.parentNode)
        .select("g.data_vertices");
    var my_data = d3.select(this).datum();

    d3.select(this).classed("data-hover", false);

    // select source gripper
    selectIOGripper(vertex_selection, my_data.source)
      .each(function(d) {
        this.dispatchEvent(new CustomEvent("mouseout"));
      });

    // select target gripper
    selectIOGripper(vertex_selection, my_data.target)
      .each(function(d) {
        this.dispatchEvent(new CustomEvent("mouseout"));
      });
  }

  IOGripperMousedownHandler(datum, index, group)
  {
    if (d3.event.button != 1)
    {
      return;
    }
    // Remove mouseover / out listeners from all gripper-groups, then add new ones
    var io_grippers = d3.select(this.svg_ref.current).selectAll(".gripper-group");

    io_grippers
      .on("mouseover", null)
      .on("mouseout", null);

    // Hide this gripper's label
    d3.select(this.svg_ref.current).selectAll(".label").datum(datum).attr("visibility", "hidden");

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
    this.dragStartPos = d3.mouse(this.viewport_ref.current);

    // Give compatible IOs a new listener and highlight them
    io_grippers
      .filter(d =>
              d.nodeName !== datum.nodeName &&
              d.kind !== datum.kind &&
              d.serialized_type === datum.serialized_type)
      .classed("compatible", true)
      .on("mouseover",
          this.IOGroupDraggingMouseoverHandler.bind(this))
      .on("mouseout",
          this.IOGroupDraggingMouseoutHandler.bind(this))
      .selectAll(".label")
      .attr("visibility", "visible");

    // Give the canvas a move and mouseup handler
    d3.select(this.viewport_ref.current)
      .on("mousemove.drag_io", this.canvasIOMoveHandler.bind(this))
      .on("mouseup.drag_io", this.canvasIOUpHandler.bind(this));

    d3.event.preventDefault();
    d3.event.stopPropagation();
  }

  canvasIOMoveHandler(d, index, group)
  {
    if ((d3.event.buttons & 1) === 0)
    {
      this.canvasIOUpHandler(d, index, group);
      return;
    }

    var drawingLine = d3.select(this.viewport_ref.current).selectAll(".drawing-indicator").data([
      {
        start: this.dragStartPos,
        end: d3.mouse(this.viewport_ref.current)
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
            this.onError("Failed to wire data " + this.nextWiringSource +
                         "to " + this.nextWiringTarget);
          }
        });
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
      .on("mouseover", this.IOGroupDefaultMouseoverHandler)
      .on("mouseout", this.IOGroupDefaultMouseoutHandler)
    // And hide the labels again
      .selectAll(".label")
      .attr("visibility", "hidden");

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

  nodeClickHandler(d, index, group)
  {
    console.log("Henlo");
  }

  nodeMousedownHandler(d, index, group)
  {
    console.log("Ohai");

    if (d3.event.button != 1)
    {
      return;
    }

    this.draggedNode = d;
  }
}

function RemoveNode(props)
{
  return (
    <div className="p-2 placeholder">
      Remove Node button, eventually.
    </div>
  );
}
class SelectedNode extends Component
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
    this.onClickAdd = this.onClickAdd.bind(this);
  }

  render()
  {
    if (this.props.node === null)
    {
      return (
        <div className="p-2 d-flex flex-column">
          No Node Selected
        </div>
      );
    }

    return(
      <div className="p-2 d-flex flex-column">
        <button className="btn btn-block btn-primary"
                disabled={!this.state.isValid}
                onClick={this.onClickAdd}>Add to Tree</button>
        <label className="pt-2 pb-2">Parent
          <select className="form-control d-block"
                  disabled={this.props.parents.length == 0}
                  ref={this.selectRef}
                  defaultValue={ (this.props.parents.length > 0) ? this.props.parents[0].name : null }>
            {this.props.parents.map(x => (<option key={x.name} value={x.name}>{x.name}</option>))}
          </select>
        </label>
        <input className="d-block"
               type="text"
               value={this.state.name}
               onChange={this.nameChangeHandler}/>
        <h4>{this.state.name}</h4>
        {this.renderParamInputs(this.state.options, 'options')}
        {/* {this.renderParamInputs(this.state.inputs, 'inputs')} */}
        {/* {this.renderParamInputs(this.state.outputs, 'outputs')} */}
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
          x.value.value["py/object"] = x.value.substring('__'.length);
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

    if (paramType.toLowerCase() == 'options')
    {
      this.setState(
        (prevState, props) =>
          {
            return {options: prevState.options.map(map_fun)};
          });
    }
    else if (paramType.toLowerCase() == 'inputs')
    {
      this.setState(
        (prevState, props) =>
          {
            return {inputs: prevState.inputs.map(map_fun)};
          });
    }
    else if (paramType.toLowerCase() == 'outputs')
    {
      this.setState(
        (prevState, props) =>
          {
            return {outputs: prevState.outputs.map(map_fun)};
          });
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
          <label>{paramItem.key}
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
            var newValue = event.target.value;
            if (isNaN(newValue))
            {
              newValue = 0;
            }
            onNewValue(newValue);
          };

      return (
        <div className="form-group">
          <label>{paramItem.key}
            <input type="number" name="integer"
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
          <label>{paramItem.key}
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
          <label>{paramItem.key}
            <input type="text"
                   className="form-control mt-2"
                   value={paramItem.value.value}
                   onChange={changeHandler}/>
          </label>
        </div>
      );
    }
    else if (valueType === 'boolean')
    {
      // Checkbox
      changeHandler = (event) =>
        {
          onNewValue(event.target.checked || false);
        };

      return (
        <div className="form-check m-1">
          <label className="form-check-label">{paramItem.key}
            <input type="checkbox"
                   className="form-check-input"
                   checked={this.state.value}
                   onChange={this.handleChange} />
          </label>
        </div>
      );
    }
    else if (valueType === 'unset_optionref')
    {
      return (
        <div className="form-group m-1">
          <label>{paramItem.key}
            <input type="text"
                   className="form-control mt-2"
                   disabled="true"
                   checked={this.state.value}/>
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
          <label>{paramItem.key}
            <JSONInput initialValue={JSON.stringify(paramItem.value.value)}
                       onValidityChange={onValidityChange}
                       onNewValue={onNewValue}/>
          </label>
        </div>
      );
    }
  }

  renderParamInputs(params, name)
  {
    var param_rows = params.map(x => {
      return (
        <tr key={name + x.key}>
          <td>
            {this.inputForValue(x, this.updateValidity, this.updateValue.bind(this, name, x.key))}
          </td>
        </tr>
      );
    });

    return (
      <div>
        <h5>{name}</h5>
        <table>
          <tbody>
            {param_rows}
          </tbody>
        </table>
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
