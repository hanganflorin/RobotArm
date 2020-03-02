import React from 'react';
import './styles/App.css';
// import SimpleSlider from "./SimpleSlider";
import SimpleSlider2 from "./SimpleSlider2";

class App extends React.Component {
    constructor(props) {
        super(props);

        this.state = {
            socket: null,
            data: null,
        };

        this.text = "";

    }
    
    componentDidMount() {
        const s = new WebSocket('ws://192.168.0.103:81/');
        this.setState({socket: s});
        s.onmessage = (event) => {
           // const json = JSON.parse(event.data.replace(/[^\x01-\x7F]/g, ""));
            // this.setState({data: event.data})
            //this.setState({data: json});
            console.log(event.data);
            // const c = event.data;
            // if (c.match(/^[a-z0-9]+$/i) !== null) // isAlphaNumeric
            //     this.text += c;
            // if ( c === "\n" ) {
            //     console.log(this.text);
            //     this.text = "";
            // }
        };
    }

    render() {
        const json = this.state.data;
        return (
            <div className="App">
                <center>
                    {/*<Led socket={this.socket}/>*/}
                    {/*<SimpleSlider code={"a"} socket={this.state.socket}/>*/}
                    {/*<SimpleSlider code={"b"} socket={this.state.socket}/>*/}
                    <SimpleSlider2 code={"P1"} value = {json ? json.p1 : 0}/>
                    <SimpleSlider2 code={"P2"} value = {json ? json.p2 : 0}/>
                    <SimpleSlider2 code={"P3"} value = {json ? json.p3 : 0}/>
                    <SimpleSlider2 code={"P4"} value = {json ? json.p4 : 0}/>
                    <SimpleSlider2 code={"P5"} value = {json ? json.p5 : 0}/>
                    {/*<h1>{json.p5 + " " + json.p4 + " " + json.p3 + " " + json.p2 + " " + json.p1}</h1>*/}
                </center>
            </div>
        );
    };
}

export default App;
