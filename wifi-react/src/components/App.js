import React from 'react';
import './styles/App.css';
import SimpleSlider from "./SimpleSlider";

class App extends React.Component {
     constructor(props) {
        super(props);

        this.state = {
            socket: null,
        };

    }

    componentDidMount() {
         this.setState({socket: new WebSocket('ws://192.168.43.249:81/') });
    }

    render() {
        return (
            <div className="App">
                <center>
                    {/*<Led socket={this.socket}/>*/}
                    <SimpleSlider code={"a"} socket={this.state.socket}/>
                    <SimpleSlider code={"b"} socket={this.state.socket}/>
                </center>
            </div>
        );
    };
}

export default App;
