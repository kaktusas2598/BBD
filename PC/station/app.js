//TODO: add to database?
//GLOBALS
var port = 8080
var portName = '/dev/ttyUSB0'

//TODO:
//Database for storing configs foraslaves, types and auto config commands
//Material design :D
//Implement protocol

//TODO: use material-ui react components with reactJs or angular material

//Socket IO for real time communication with server
var socket_io = require('socket.io');
//Node SerialPort for communicating with Environment Control Station
var serialport = require('serialport');

var express = require('express');
var path = require('path');
var favicon = require('serve-favicon')
var logger = require('morgan');
var cookieParser = require('cookie-parser');
var bodyParser = require('body-parser');

//Yay
var db = require('./model/db');

var app = express();

// Socket.io
var io = socket_io();
app.io = io;

//var routes = require('./routes/index')(io);
var routes = require('./routes/index');
var users = require('./routes/users');
//TODO: create login.js and create routes for authentication, using npm passport
var login = require('./routes/login');

//var devices = require('./model/devices');


// Serial port initialization:
var  SerialPort  = serialport.SerialPort,            // make a local instance of serial
    //portName = process.argv[2],                             // get the port name from the command line
    portConfig = {
        baudRate: 9600,
        // call myPort.on('data') when a newline is received:
        //parser: serialport.parsers.readline('\n')
};
serialport.list(function (err, ports) {
	app.set('ports',ports);
});


// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'jade');

// uncomment after placing your favicon in /public
//app.use(favicon(path.join(__dirname, 'public', 'favicon.ico')));
app.use(logger('dev'));
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: false }));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, 'public')));

app.use('/', routes);
app.use('/users', users);
app.use('/login', login);

io.on('connection', openSocket);  // listener for websocket data
function openSocket(socket){
    console.log('New user address: ' + socket.handshake.address);
    // send something to the web client with the data:
    socket.emit('message', 'Hello, ' + socket.handshake.address);

    // this function runs if there's input from the client:
    socket.on('message', function(data) {
        //myPort.write(data);                         // send the data to the serial device
    });

    // this function runs if there's input from the serialport:
    /*myPort.on('data', function(data) {
        socket.emit('message', data);       // send the data to the client
    });*/
}

// catch 404 and forward to error handler
app.use(function(req, res, next) {
  var err = new Error('Not Found');
  err.status = 404;
  next(err);
});

// error handlers

// development error handler
// will print stacktrace
if (app.get('env') === 'development') {
  app.use(function(err, req, res, next) {
    res.status(err.status || 500);
    res.render('error', {
      message: err.message,
      error: err
    });
  });
}

// production error handler
// no stacktraces leaked to user
app.use(function(err, req, res, next) {
  res.status(err.status || 500);
  req.serialport = serialport;

  res.render('error', {
    message: err.message,
    error: {}
  });
});


app.serialport = serialport;
module.exports = app;
