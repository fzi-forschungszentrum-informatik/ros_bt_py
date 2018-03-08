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

function onTreeUpdate(tree_msg) {
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
  // animate to actual position
      .attr("transform", function(d) {
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

function init() {
  var ros = new ROSLIB.Ros({
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
