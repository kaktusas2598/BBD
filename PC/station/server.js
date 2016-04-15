//GLOBALS
var port = 8080
var portName = '/dev/ttyUSB0'

//TODO:
//Actual web app!!!
//Database for storing configs foraslaves, types and auto config commands
//Material design :D
//Implement protocol

//TODO: use node serialport module
//TODO: use material-ui react components for material design

//INIT WEB APPLICATION AND SOCKET.IO SERVER
var express = require('express');
var io = require('socket.io');
var app = express();           // make an instance of express.js
var server = app.listen(port);  // start a server with the express instance
var socketServer = io(server);  // make a socket server using the express server

// Serial port initialization:
var serialport = require('serialport'),          // include the serialport library
    SerialPort  = serialport.SerialPort,            // make a local instance of serial
    //portName = process.argv[2],                             // get the port name from the command line
    portConfig = {
        baudRate: 9600,
        // call myPort.on('data') when a newline is received:
        //parser: serialport.parsers.readline('\n')
    };

//ExpressJS settings
app.set('view engine', 'jade');
//  set up server and socketServer listener functions:
app.use(express.static('public'));                  // serve files from the public folder
app.get('/:name', serveFiles);                          // listener for all static file requests
socketServer.on('connection', openSocket);  // listener for websocket data

// open the serial port:
var myPort = new SerialPort(portName, portConfig,false);//TODO, clear 3rd argument, if you want to open port immediately

app.get('/', function (req, res) {
	  res.render('index', {});
});

function serveFiles(request, response) {
    var fileName = request.params.name;             // get the file name from the request
    response.sendFile(fileName);                            // send the file
}

function openSocket(socket){
    console.log('new user address: ' + socket.handshake.address);
    // send something to the web client with the data:
    socket.emit('message', 'Hello, ' + socket.handshake.address);

    // this function runs if there's input from the client:
    socket.on('message', function(data) {
        myPort.write(data);                         // send the data to the serial device
    });

    //List serial ports
    serialport.list(function (err, ports) {
        ports.forEach(function(port) {
            //TODO: webify!
            console.log(port.comName);
        });
    });

    // this function runs if there's input from the serialport:
    myPort.on('data', function(data) {
        socket.emit('message', data);       // send the data to the client
    });
}



//console.log(serialPort);
/*var serial= new serialPort.SerialPort("/dev/ttyUSB0", {
    baudrate: 9600
},false);//Open immediately flag to false

//RESOURCE: https://nodejs.org/api/buffer.html
const requestBuffer = new Buffer("96150100A9","hex");

//RESOURCE: https://github.com/voodootikigod/node-serialport
serial.open(function (error) {
    if ( error ) {
        console.log('failed to open: '+error);
    } else {
        console.log('open');
        serial.on('data', function(data) {
            console.log('Data received: \n' + data.toString('hex'));
        });
        //serial.write(0x61, function(err, results) {
        //console.log('err ' + err);
        //console.log('results ' + results);
        //});
        serial.write(requestBuffer, function(err,results){
            serial.drain();
            //console.log(Buffer.from(results,'hex'));
            //console.log('results ' + results);
            console.log('Results: \n' + results.toString('hex'));
        });

    }
});
*/

