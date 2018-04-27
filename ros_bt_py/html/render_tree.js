function render() {
  var tree_msg = {
    "nodes": [
      {
        "name": "Foo",
        "state": "IDLE",
        "child_names": [
          "Bar",
          "Baz"
        ]
      },
      {
        "name": "Bar",
        "state": "RUNNING",
        "child_names": ["a"]
      },
      {
        "name": "a",
        "state": "SUCCEEDED",
      },
      {
        "name": "Baz",
        "state": "FAILED",
      }
    ]
  }

  onTreeUpdate(tree_msg);
}

function delete_node() {
  var node_name = document.getElementById("node_delete_selector").value;
  new ROSLIB.Service({
    ros: ros,
    name: '/tree_node/remove_node',
    serviceType: 'ros_bt_py_msgs/RemoveNode'
  }).callService(
    new ROSLIB.ServiceRequest({
      node_name: node_name,
      remove_children: document.getElementById("node_delete_children").checked
    }),
    function(response) {
      if (response.success) {
        console.log('removed node ' + node_name + ' successfully');
      }
      else {
        console.log('failed to remove node ' + node_name + ' : ' + response.error_message);
      }
    });
}

function add_node() {
  var node_map = {
    'sequence': {
      is_subtree: false,
      module: 'ros_bt_py.nodes.sequence',
      node_class: 'Sequence',
      name: 'sequence',
      child_names: []
    },
    'fallback': {
      is_subtree: false,
      module: 'ros_bt_py.nodes.fallback',
      node_class: 'Fallback',
      name: 'fallback',
      child_names: []
    },
    'succeeder': {
      is_subtree: false,
      module: 'ros_bt_py.nodes.mock_nodes',
      node_class: 'MockLeaf',
      name: 'succeeder',
      options: [
        {
          key: 'output_type',
          serialized_value: '{"py/type": "__builtin__.str"}'
        },
        {
          key: 'state_values',
          serialized_value: '["SUCCEEDED"]'
        },
        {
          key: 'output_values',
          serialized_value: '["Yay!"]'
        },
      ],
      child_names: []
    },
    'failer': {
      is_subtree: false,
      module: 'ros_bt_py.nodes.mock_nodes',
      node_class: 'MockLeaf',
      name: 'failer',
      options: [
        {
          key: 'output_type',
          serialized_value: '{"py/type": "__builtin__.str"}'
        },
        {
          key: 'state_values',
          serialized_value: '["FAILED"]'
        },
        {
          key: 'output_values',
          serialized_value: '["Noooo :("]'
        },
      ],
      child_names: []
    }
  };

  var node_msg = node_map[document.getElementById('node_type').value];

  new ROSLIB.Service({
    ros: ros,
    name: '/tree_node/add_node',
    serviceType: 'ros_bt_py_msgs/AddNode'
  }).callService(
    new ROSLIB.ServiceRequest({
      tree_name: '',
      parent_name: document.getElementById('new_node_parents').value,
      node: node_msg,
      allow_rename: true
    }),
    function(response) {
      if (response.success) {
        console.log('Added node to tree as ' + response.actual_node_name);
      }
      else {
        console.log('Failed to add node ' + node_msg.name + ': '
                    + response.error_message);
      }
    });
}

function onTreeUpdate(tree_msg) {
  // Disable all editing buttons unless tree is in EDITABLE state.
  d3.select("#add_node_btn")
    .attr("disabled", tree_msg.state === "EDITABLE" ? null : true);
  d3.select("#delete_node_btn")
    .attr("disabled", tree_msg.state === "EDITABLE" ? null : true);

  // Update the node selection dropdowns
  var add_options = function(id) {
    var parents = d3.select(id)
        .selectAll("option")
        .data(tree_msg.nodes, node => node.name);
    parents.enter()
      .append("option")
      .attr("value", d => d.name)
      .text(d => d.name);
    parents.exit().remove();
  }
  add_options("#new_node_parents");
  add_options("#node_delete_selector");

  // Update the visual tree
  var parents = {};
  var node_dict = {};
  // Find parents for all nodes once
  for (var i in tree_msg.nodes) {
    var node = tree_msg.nodes[i];
    node_dict[node.name] = node;
    for (var j in node.child_names) {
      parents[node.child_names[j]] = node.name;
    }
  }
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

  var svg = d3.select("svg"),
      width = +svg.attr("width"),
      height = +svg.attr("height");
  var g_edge = svg.selectAll("g.edges").data([null]);
  g_edge = g_edge
    .enter()
    .append("g")
    .attr("class", "edges")
    .attr("transform", "translate(0,40)")
    .merge(g_edge);

  var g_vertex = svg.selectAll("g.vertices").data([null]);
  g_vertex = g_vertex
    .enter()
    .append("g")
    .attr("class", "vertices")
    .attr("transform", "translate(0,40)")
    .merge(g_vertex);

  var node = g_vertex
      .selectAll(".node")
      .data(root.descendants(), function(node) {
        return node.id;
      });

  node.exit().remove();

  var nodeEnter = node.enter();

  drawNodes(nodeEnter);

  // node = nodeEnter.merge(node);
  node = g_vertex
    .selectAll(".btnode")
    .data(root.descendants(), function(node) {
      return node.id;
    });

  updateNodes(node);

  // Find the maximum size of all the nodes, for layout purposes
  var max_size = [0,0];
  g_vertex.selectAll('.btnode').data(root.descendants(), d => d.id)
    .each(function(d, index){
      var rect = this.getBoundingClientRect();
      d._size = rect;
      max_size[0] = Math.max(max_size[0], rect.width);
      max_size[1] = Math.max(max_size[1], rect.height);
      this.parentElement.setAttribute('width', rect.width);
      this.parentElement.setAttribute('height', rect.height);
    });
  //max_size = [max_size[0] + 30, max_size[1] + 30];
  // Calculate node positions

  var tree_size = [width - max_size[0], height - (40 + max_size[1])];

  var tree = d3.tree()
      .size(tree_size)
  //.nodeSize(max_size)
  (root);

  // Move new nodes to their starting positions
  g_vertex.selectAll(".node").data(root.descendants(), d => d.id)
    .filter(d => d._entering)
    .attr("transform", function(d) {
      // Start at parent position
      var p = findExistingParent(d);
      return "translate(" + Math.round(p.x) + "," + Math.round(p.y) + ") scale(0.1)";
    });

  var link = g_edge.selectAll(".link")
      .data(tree.links(), function(d) { return '' + d.source.id + d.target.id; });
  link.exit().remove();

  link = link
    .enter().append("path")
    .attr("class", "link")
    .attr("d", d3.linkVertical()
          .source(function(d) {
            var parent = findExistingParent(d.source);
            return [Math.round(parent.x), Math.round(parent.y + parent._size.height)];
          })
          .target(function(d) {
            var parent = findExistingParent(d.target);
            return [Math.round(parent.x), Math.round(parent.y)];
          }))
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
};

var drawNodes = function(selection) {
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
};

var updateNodes = function(selection) {
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
    });
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

  fillTables(tables.select("tbody"));
};

var fillTables = function(tbody) {
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
};

function step() {
  new ROSLIB.Service({
    ros: ros,
    name: '/tree_node/debug/continue',
    serviceType: 'ros_bt_py_msgs/Continue'
  }).callService(
    new ROSLIB.ServiceRequest({}),
    function(response) {
      if (response.success) {
        console.log('stepped successfully');
      }
      else {
        console.log('stepping failed');
      }
    });
}
function tick() {
  new ROSLIB.Service({
    ros: ros,
    name: '/tree_node/control_tree_execution',
    serviceType: 'ros_bt_py_msgs/ControlTreeExecution'
  }).callService(
    new ROSLIB.ServiceRequest({
      command: 1 // TICK_ONCE
    }),
    function(response) {
      if (response.success) {
        console.log('ticked successfully');
      }
      else {
        console.log('single tick failed');
      }
    });
}

function tick_periodically() {
  new ROSLIB.Service({
    ros: ros,
    name: '/tree_node/control_tree_execution',
    serviceType: 'ros_bt_py_msgs/ControlTreeExecution'
  }).callService(
    new ROSLIB.ServiceRequest({
      command: 2, // TICK_PERIODICALLY
      tick_frequency_hz: 0.5
    }),
    function(response) {
      if (response.success) {
        console.log('started ticking successfully');
      }
      else {
        console.log('starting periodic tick failed');
      }
    });
}

function stop_tick() {
  new ROSLIB.Service({
    ros: ros,
    name: '/tree_node/control_tree_execution',
    serviceType: 'ros_bt_py_msgs/ControlTreeExecution'
  }).callService(
    new ROSLIB.ServiceRequest({
      command: 3 // STOP
    }),
    function(response) {
      if (response.success) {
        console.log('stopped successfully');
      }
      else {
        console.log('stopping failed');
      }
    });
}

function shutdown() {
  new ROSLIB.Service({
    ros: ros,
    name: '/tree_node/control_tree_execution',
    serviceType: 'ros_bt_py_msgs/ControlTreeExecution'
  }).callService(
    new ROSLIB.ServiceRequest({
      command: 5 // SHUTDOWN
    }),
    function(response) {
      if (response.success) {
        console.log('shutdown successful');
      }
      else {
        console.log('shutdown failed');
      }
    });
}

function reset() {
  new ROSLIB.Service({
    ros: ros,
    name: '/tree_node/control_tree_execution',
    serviceType: 'ros_bt_py_msgs/ControlTreeExecution'
  }).callService(
    new ROSLIB.ServiceRequest({
      command: 4 // RESET
    }),
    function(response) {
      if (response.success) {
        console.log('reset successful');
      }
      else {
        console.log('reset failed');
      }
    });
}

function step() {
  new ROSLIB.Service({
    ros: ros,
    name: '/tree_node/debug/continue',
    serviceType: 'ros_bt_py_msgs/Continue'
  }).callService(
    new ROSLIB.ServiceRequest({}),
    function(response) {
      if (response.success) {
        console.log('stepped successfully');
      }
      else {
        console.log('stepping failed');
      }
    });
}

function change_debug_mode(enable) {
  new ROSLIB.Service({
    ros: ros,
    name: '/tree_node/debug/set_execution_mode',
    serviceType: 'ros_bt_py_msgs/SetExecutionMode'
  }).callService(
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
}

function init() {
  window.ros = new ROSLIB.Ros({
    url : 'ws://10.211.55.3:9090'
  });
  ros.on('connection', function() {
    var listener = new ROSLIB.Topic({
      ros : ros,
      name : '/tree',
      messageType : 'ros_bt_py_msgs/Tree'
    });
    // Then we add a callback to be called every time a message is published on this topic.
    listener.subscribe(onTreeUpdate);
    console.log('Connected & subscribed');
  });
}

function findExistingParent(d) {
  while (d._entering && d.parent) {
    d = d.parent;
  }
  return d;
}

