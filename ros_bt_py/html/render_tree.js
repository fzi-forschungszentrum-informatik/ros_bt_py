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
  var root = d3.stratify()
      .id(function(node) {
        return node.name;
      })
      .parentId(function(node) {
        // undefined if it has no parent - does that break the layout?
        return parents[node.name];
      })(tree_msg.nodes);

  root.sort(function(a, b) {
    if (a.height != b.height) {
      return a.height - b.height;
    }
    if (a.parent != b.parent) {
      console.log("shouldn't happen");
    }
    var child_list = a.parent.data.child_names;
    return (child_list.findIndex(x => x == a.data.name) 
            - child_list.findIndex(x => x == b.data.name));
  });


  var svg = d3.select("svg"),
      width = +svg.attr("width"),
      height = +svg.attr("height");
  var g = svg.selectAll("g")
      .data([null]);
  g = g.enter()
        .append("g")
        .attr("transform", "translate(0,40)")
      .merge(g);

  var tree = d3.tree()
      .size([width - 160, height - 160]);
  var link = g.selectAll(".link")
      .data(tree(root).links())
      .enter().append("path")
      .attr("class", "link")
      .attr("d", d3.linkVertical()
            .x(function(d) {
              return d.x;
            })
            .y(function(d) {
              return d.y;
            }));

  var node = g.selectAll(".node")
      .data(root.descendants(), function(node) {return node.id;})
      .enter().append("g")
      .attr("class", function(d) {
        return "node" + (d.children ? " node--internal" : " node--leaf");
      })
      .attr("transform", function(d) {
        return "translate(" + d.x + "," + d.y + ")";
      })

  node.append("circle")
    .attr("r", 5.0)
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

  node.append("text")
    .attr("dy", 3)
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
      return d.id.substring(d.id.lastIndexOf(".") + 1);
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
  });
}
