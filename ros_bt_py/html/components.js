
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
      showDataGraph: true
    };
    this.ros = new ROSLIB.Ros({
      url : this.state.ros_uri
    });

    // Bind these here so this works as expected in callbacks
    this.onError = this.onError.bind(this);
    this.onSelectionChange = this.onSelectionChange.bind(this);
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
                  <button className="btn btn-primary btn-block"
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
                      node={this.state.selected_node} />
                  </div>
                  <div className="col">
                    <AddNode ros={this.ros}
                             bt_namespace={this.state.bt_namespace}
                             onError={this.onError}/>
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
    <header id="header" className="d-flex flex-column flex-md-row align-items-center placeholder">
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
      tree_msg: null
    };

    this.horizontal_spacing = 40;
    this.vertical_spacing = 25;

    this.io_gripper_spacing = 10;
    this.max_io_gripper_size = 15;

    this.tree_topic = new ROSLIB.Topic({
      ros : props.ros,
      name : props.bt_namespace + 'tree',
      messageType : 'ros_bt_py_msgs/Tree'
    });

    this.svg_ref = createRef();
    this.viewport_ref = createRef();

    this.onSelectionChange = props.onSelectionChange;
    this.onError = props.onError;

    this.onTreeUpdate = this.onTreeUpdate.bind(this);
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
        <g id="container" ref={this.svg_ref}/></svg>
    );
  }

  onTreeUpdate(tree_msg)
  {
    this.drawEverything(tree_msg);

    // Disable all interaction (except for zooming and panning) when
    // the tree isn't editable
    if (tree_msg.state !== "EDITABLE")
    {

    }

    // Hide or show data graph
    if (this.props.showDataGraph)
    {
      d3.select(this.svg_ref).attr("visibility", "hidden");
    }
    else
    {
      d3.select(this.svg_ref).attr("visibility", "visible");
    }
  }

  drawEverything(tree_msg)
  {
    var forest_root = {
      "name": "__forest_root",
      "child_names": []
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
        child_list.findIndex(x => x == a.data.name) -
          child_list.findIndex(x => x == b.data.name)
      );
    });


    var svg = d3.select(this.svg_ref.current);


    var container = d3.select(this.viewport_ref.current);
    var width = container.attr("width"),
        height = container.attr("height");


    var g_edge = svg.selectAll("g.edges").data([null]);
    g_edge = g_edge
      .enter()
      .append("g")
      .attr("class", "edges")
      .merge(g_edge);

    var g_vertex = svg.selectAll("g.vertices").data([null]);
    g_vertex = g_vertex
      .enter()
      .append("g")
      .attr("class", "vertices")
      .merge(g_vertex);

    var g_data = svg.selectAll("g.data_graph").data([null]);
    g_data = g_data
      .enter()
      .append("g")
      .attr("class", "data_graph")
      .merge(g_data);

    var g_droptargets = svg.selectAll("g.drop_targets").data([null]);
    g_droptargets = g_droptargets
      .enter()
      .append("g")
      .attr("class", "drop_targets")
      .merge(g_droptargets);

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
        console.log('#####');
        console.log(this);
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

    this.drawDataGraph(g_data, node.data());

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

  drawNodes(selection) {
    selection.each(function(d) {
      d._entering = true;
      d._show = true;
      d.x = 0;
      d.y = 0;
    });

    var fo = selection.append('foreignObject')
        .attr("class", function(d) {
          return "node" + (d.children ? " node--internal" : " node--leaf");
        });

    var div = fo
        .append("xhtml:body")
        .attr("class", "btnode");
  }

  updateNodes(selection) {
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

  fillTables (tbody) {
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

  findExistingParent(d) {
    while (d._entering && d.parent) {
      d = d.parent;
    }
    return d;
  }

  drawDataGraph(g_data, data)
  {
    var max_io_gripper_size = this.max_io_gripper_size;
    var io_gripper_spacing = this.io_gripper_spacing;
    var io_groups = g_data
        .selectAll('.io_group').data(
          data.map(
            d => {
              var inputs = d.data.inputs || [];
              var outputs = d.data.outputs || [];
              var i = inputs.length;
              var o = outputs.length;
              var gripper_size = Math.min(
                max_io_gripper_size,
                // +1 to ensure nice padding on the bottom
                (d._size.height - ((o + 1) * io_gripper_spacing)) / o,
                (d._size.height - ((i + 1) * io_gripper_spacing)) / i);
              return {
                name: d.data.name,
                x: d.x,
                y: d.y,
                width: d._size.width,
                height: d._size.height,
                gripper_size: gripper_size,
                inputs: inputs,
                outputs: outputs,
              };
            }),
          d => d.name);
    io_groups.exit().remove();
    io_groups = io_groups
      .enter()
      .append("g")
      .attr("class", "io_group")
      .merge(io_groups);

    io_groups
      .attr("transform", function(d) {
        return "translate(" + Math.round(d.x) + ", " + Math.round(d.y) + ")";
      });

      // .attr("dx", d => d.x)
      // .attr("dy", d => d.y);

    var inputs = io_groups.selectAll(".input_grabber").data(
      function(d) {
        return d.inputs.map(function(el, index) {
          el.rel_x = -1.0 * (d.gripper_size + 0.5 * d.width);
          el.rel_y = io_gripper_spacing + (index * (io_gripper_spacing + d.gripper_size));
          el.gripper_size = d.gripper_size;
          return el;
        });
      },
      d => d.key);
    inputs.exit().remove();
    inputs = inputs
      .enter()
      .append("g")
      .attr("class", "input_gripper_group")
      .merge(inputs);
    inputs
      .attr("transform", function(d) {
        return "translate(" + Math.round(d.rel_x) + ", " + Math.round(d.rel_y) + ")";
      });

    var input_grippers = inputs.selectAll(".gripper").data(d=> [d]);
    input_grippers.exit().remove();
    input_grippers = input_grippers
      .enter()
      .append("rect")
      .attr("class", "gripper input-gripper")
      .attr("width", d => d.gripper_size)
      .attr("height", d => d.gripper_size)
      .merge(input_grippers);

    var input_labels = inputs.selectAll(".label").data(d=> [d]);
    input_labels.exit().remove();
    input_labels = input_labels
      .enter()
      .append("text")
      .attr("class", "label")
      .attr("text-anchor", "end")
      .attr("dominant-baseline", "middle")
      .attr("dx", d => Math.round(-5))
      .attr("dy", d => Math.round(0.5 * d.gripper_size))
      .merge(input_labels);
    input_labels.text(d => d.key);


    // Same thing for the outputs, except they're at the right end of the node

    var outputs = io_groups.selectAll(".output_grabber").data(
      function(d) {
        return d.outputs.map(function(el, index) {
          el.rel_x = 0.5 * d.width;
          el.rel_y = io_gripper_spacing + (index * (io_gripper_spacing + d.gripper_size));
          el.gripper_size = d.gripper_size;
          return el;
        });
      },
      d => d.key);
    outputs.exit().remove();
    outputs = outputs
      .enter()
      .append("g")
      .attr("class", "output_gripper_group")
      .merge(outputs);
    outputs
      .attr("transform", function(d) {
        return "translate(" + Math.round(d.rel_x) + ", " + Math.round(d.rel_y) + ")";
      });

    var output_grippers = outputs.selectAll(".gripper").data(d=> [d]);
    output_grippers.exit().remove();
    output_grippers = output_grippers
      .enter()
      .append("rect")
      .attr("class", "gripper output-gripper")
      .attr("width", d => d.gripper_size)
      .attr("height", d => d.gripper_size)
      .merge(output_grippers);

    var output_labels = outputs.selectAll(".label").data(d=> [d]);
    output_labels.exit().remove();
    output_labels = output_labels
      .enter()
      .append("text")
      .attr("class", "label")
      .attr("text-anchor", "start")
      .attr("dominant-baseline", "middle")
      .attr("dx", d => Math.round(d.gripper_size + 5))
      .attr("dy", d => Math.round(0.5 * d.gripper_size))
      .merge(output_labels);
    output_labels.text(d => d.key);
  }
}

function AddNode(props)
{
  return (
    <div className="p-2 placeholder">
      Add Node button, eventually.
    </div>
  );
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

    this.nameChangeHandler = this.nameChangeHandler.bind(this);
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
                onClick={this.onClickAdd}>Add to Tree</button>
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

  onClickAdd()
  {
    var msg = this.buildNodeMessage();
    console.log('trying to add node to tree:');
    console.log(msg);
    this.add_node_service.callService(
      new ROSLIB.ServiceRequest({
        tree_name: '',
        parent_name: '',
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

  inputForValue(paramItem, onNewValue)
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
            {this.inputForValue(x, this.updateValue.bind(this, name, x.key))}
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
    var is_valid = false;
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
    }
    catch(e)
    {
      // Do nothing
      this.setState({is_valid: false});
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
