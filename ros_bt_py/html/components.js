
// import { Component } from 'preact';
// import { render } from 'preact';
// import { createRef } from 'preact';
var Component = React.Component;
var render = ReactDOM.render;
var createRef = React.createRef;
var Fragment = React.Fragment; //'x-fragment';

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
      <div className="io_values">
        <h5>{title}</h5>
        <table><tbody>{rows}</tbody></table>
      </div>
    );
  };

  render() {
    return (
      <div className="box">
        <h4 className="node_class">{this.props.node.node_class}</h4>
        <h5 className="node_module">{this.props.node.module}</h5>
        { this.renderIOTable(this.props.node.options, 'Options') }
        { this.renderIOTable(this.props.node.inputs, 'Inputs') }
        { this.renderIOTable(this.props.node.outputs, 'Outputs') }
      </div>
    );
  };
}

class NodeList extends Component
{
  constructor(props)
  {
    super(props);

    this.state = {available_nodes: []};

    this.onError = props.onError;
    this.get_nodes_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'get_available_nodes',
      serviceType: 'ros_bt_py_msgs/GetAvailableNodes'
    });
  }

  componentDidMount()
  {
    this.get_nodes_service.callService(
      new ROSLIB.ServiceRequest({
        node_modules: ['']
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

  render()
  {
    var items = this.state.available_nodes.map( (node) => {
      return (<NodeListItem node={node} key={node.module + node.node_class} />);
    });
    return(
      <div className="box vertical_list">
        {items}
      </div>);
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
      selected_node: null
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
                                          onError={this.onError}/>
                  </div>
                </div>

                <div className="row">
                  <div className="col">
                    <SelectedNode selected={this.state.selected_node} />
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
    <header id="header" className="d-flex flex-column flex-md-row placeholder">
      <DebugControls
        ros={props.ros} onError={props.onError}/>
      <TickControls
        ros={props.ros} onError={props.onError}/>
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
      <header id="header" className="d-flex flex-column flex-md-row">
        <button onClick={this.controlExec.bind(this, 1)}>Tick Once</button>
        <button onClick={this.controlExec.bind(this, 2)}>Tick Periodically</button>
        <button onClick={this.controlExec.bind(this, 4)}>Stop</button>
        <button onClick={this.controlExec.bind(this, 5)}>Reset</button>
        <button onClick={this.controlExec.bind(this, 6)}>Shutdown</button>
      </header>
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

    this.tick_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'control_tree_execution',
      serviceType: 'ros_bt_py_msgs/ControlTreeExecution'
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
    this.setState({value: event.target.value});
  }

  render()
  {
    return (
      <div>
        <div>
          <input type="checkbox"
                 id="debugging"
                 value={this.state.value}
                 onChange={this.handleChange} />
          <label htmlFor="debugging">Debug</label>
        </div>
        <button onClick={this.onClickStep}>Step</button>
      </div>
    );
  }
}

class D3BehaviorTreeEditor extends Component
{
  constructor(props)
  {
    super(props);

    this.horizontal_spacing = 10;
    this.vertical_spacing = 25;

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
    console.log("Current zoom level is: " + zoom);
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
    console.log(root);
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
        .append("xhtml:div")
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
function SelectedNode(props)
{
  return (
    <div className="p-2 placeholder">
      Selected Node, eventually.
    </div>
  );
}

render(<App />, document.body);
