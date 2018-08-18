
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

class NodeListItem extends React.Component {
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

class NodeList extends React.Component
{
  render()
  {
    var items = this.props.available_nodes.map( (node) => {
      return (<NodeListItem node={node} key={node.module + node.node_class} />);
    });
    return(
        <div className="box vertical_list">
        {items}
      </div>);
  }
}
