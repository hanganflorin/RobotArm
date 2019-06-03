import React from 'react';
import Button from '@material-ui/core/Button';
import './styles/Led.css'

class Led extends React.Component {
    constructor(props) {
        super(props);

        this.state = {
            value: false,
        };

       this.onClickHandler = this.onClickHandler.bind(this);
    }

    componentDidMount() {
        // fetch('https://api.mydomain.com')
        //   .then(response => response.json())
        //   .then(data => this.setState({ data }));
    }

    render() {
        const text = this.state.value ? "ON" : "OFF";

        return (
            <div className="buttonMargin">
                <h1>{text}</h1>
                <Button onClick={this.onClickHandler} variant="contained" color="primary">Toggle</Button>
            </div>
        );
    }

    onClickHandler() {
        // const {socket} = this.props;
        // const {value} = this.state;
        // const text = value ? "off" : "on";
        //
        // socket.send("a"+)

        // fetch('http://192.168.43.249/api/led?value=' + text, {'mode': 'no-cors'})
        //     .then(() => this.setState({value: !value}));


    }

}

export default Led;
