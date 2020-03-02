import React from 'react';
import Slider from '@material-ui/lab/Slider';

import './styles/SimpleSlider.css'

class SimpleSlider2 extends React.Component {
    constructor(props) {
        super(props);

        this.state = {
            value: 0,
        };

        //this.handleChange = this.handleChange.bind(this);
    }

    componentDidMount() {
        // fetch('https://api.mydomain.com')
        //   .then(response => response.json())
        //   .then(data => this.setState({ data }));
    }

    render() {
        const {value} = this.props;

        return (
            <div className="sliderContainer">
                <Slider
                    className="sliderClass"
                    value={value}
                    min={0}
                    max={180}
                    step={1}
                    aria-labelledby="label"
                    //onChange={this.handleChange}
                />
                <h1>{this.props.code} {value}</h1>
            </div>
        );
    }

    // handleChange(event, value) {
    //     const {socket, code} = this.props;
    //     this.setState({value});
    //     socket.send(code+value);
    //     //fetch('http://192.168.43.249/api/servo?value=' + value, {'mode': 'no-cors'});
    // }

}

export default SimpleSlider2;
