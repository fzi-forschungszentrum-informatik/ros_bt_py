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
      node: node_msg
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
  var root = d3.stratify()
      .id(function(node) {
        return node.name;
      })
      .parentId(function(node) {
        // undefined if it has no parent - does that break the layout?
        if (node.name in parents) {
          return parents[node.name];
        }
        else if (node.name === forest_root.name) {
          return undefined;
        }
        else {
          forest_root.child_names.push(node.name);
          return forest_root.name;
        }
      })(tree_msg.nodes);

  root.sort(function(a, b) {
    if (a.height !== b.height) {
      return a.height - b.height;
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
    return (child_list.findIndex(x => x == a.data.name)
            - child_list.findIndex(x => x == b.data.name));
  });


  var svg = d3.select("svg"),
      width = +svg.attr("width"),
      height = +svg.attr("height");
  var g_edge = svg.selectAll("g.edges")
      .data([null]);
  g_edge = g_edge.enter()
    .append("g")
    .attr("class", "edges")
    .attr("transform", "translate(0,40)")
    .merge(g_edge);

  var g_vertex = svg.selectAll("g.vertices")
      .data([null]);
  g_vertex = g_vertex.enter()
    .append("g")
    .attr("class", "vertices")
    .attr("transform", "translate(0,40)")
    .merge(g_vertex);


  var findExistingParent = function(d) {
    while(d._entering && d.parent) {
      d = d.parent;
    }
    return d;
  };

  var tree = d3.tree()
      .size([width - 160, height - 160])(root);

  var link = g_edge.selectAll(".link")
      .data(tree.links(), function(d) { return '' + d.source.id + d.target.id; });
  link.exit().remove();
  link.enter().each(function(d) {
    d.source._entering = true;
    d.target._entering = true;
  });

  link = link
      .enter().append("path")
      .attr("class", "link")
      .attr("d", d3.linkVertical()
            .x(function(d) {
              return findExistingParent(d).x;
            })
            .y(function(d) {
              return findExistingParent(d).y;
            }))
    .merge(link);

  link.each(function(d) {
    d.source._entering = false;
    d.target._entering = false;
  });

  link.transition()
    .duration(200).
    attr("d", d3.linkVertical()
         .x(function(d) {
           return d.x;
         })
         .y(function(d) {
           return d.y;
         }));


  var node = g_vertex.selectAll(".node")
      .data(root.descendants(), function(node) {return node.id;});

  node.exit().remove();

  var nodeEnter = node
      .enter();

  nodeEnter.each(function(d) {
    d._entering = true;
  });
  var nodeG = nodeEnter.append("g")
    .attr("class", function(d) {
      return "node" + (d.children ? " node--internal" : " node--leaf");
    })
    .attr("transform", function(d) {
      // Start at parent position
      var p = findExistingParent(d);
      return "translate(" + p.x + "," + p.y + ")";
    });
  nodeG.append("circle")
    .attr("r", 5.0);

  nodeG.append("text")
      .attr("dy", 3);

  node = nodeEnter.merge(node);
  node.each(function(d) {
    d._entering = false;
  });

  // new selection, now with the elements we just added with enter()
  // above
  node = g_vertex.selectAll(".node")
    .data(root.descendants(), function(node) {return node.id;})
    .transition()
    .duration(200)
      .attr("transform", function(d) {
        // animate to actual position
        return "translate(" + d.x + "," + d.y + ")";
      })

  node.select("circle")
    .style("fill", function(d) {
      switch (d.data.state){
  	  case "SUCCEEDED":
        return "#090";
      case "FAILED":
        return "#900";
      case "RUNNING":
    	  return "#990";
      case "IDLE":
    	  return "#009";
      default:
    	  return "#999";
      }
    });

  node.select("text")
    .attr("x", function(d) {
      return d.children ? -16 : 0;
    })
    .attr("y", function(d) {
      return d.children ? 0 : 20;
    })
    .style("text-anchor", function(d) {
      return d.children ? "end" : "middle";
    })
    .text(function(d) {
      return d.id + ': ' + d.data.state;
    });

  console.log(root);
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
