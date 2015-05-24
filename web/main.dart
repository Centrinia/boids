// Copyright (c) 2015, <your name>. All rights reserved. Use of this source code
// is governed by a BSD-style license that can be found in the LICENSE file.

import 'dart:math';
import 'dart:html';
import 'dart:async';
import 'package:vector_math/vector_math.dart';

class Boid {
  Vector2 _position;
  Vector2 _velocity;

  Boid(Vector2 x) {
    _position = new Vector2.copy(x);
    _velocity = new Vector2(0.0,0.0);
  }
  get position => _position;
  get momentum => _velocity;

  set position(Vector2 x) {
    _position = new Vector2.copy(x);
  }
  set momentum(Vector2 v) {
    _velocity = new Vector2.copy(v);
  }
}
class Renderer {
  static const UNIT_SIZE = 10000.0;
  static final SCREEN_CENTER = new Vector2(10.0, 10.0);
  static const TICS_PER_SECOND = 35;
  static const BOID_RADIUS = 20.0;
  static const FLYING_VELOCITY = 1.0 / 400.0;
  static const ATTRACTION_VELOCITY = 1.0 / 50.0;
  static const AVOIDANCE_VELOCITY = 1.0;
  static const FLYING_ACCELERATION = 1.0 / 40.0;
  static const MINIMUM_DISTANCE = 30.0;
  static const MAXIMUM_VELOCITY = 100.0;
  CanvasElement _canvas;
  CanvasRenderingContext2D _context;

  List<Boid> _boids;
  List<Boid> _boidsBackup;
  
  Vector2 _target = null;

  Renderer(CanvasElement canvas) {
    const BOID_COUNT = 50;
    _canvas = canvas;
    _context = _canvas.context2D;

    _boids = new List<Boid>(BOID_COUNT);
    _boidsBackup = new List<Boid>(BOID_COUNT);
    Random rng = new Random();
    Vector2 c = new Vector2(0.5,0.5);
    for(int i = 0;i<_boids.length;i++){
      _boids[i] = new Boid((new Vector2(rng.nextDouble(), rng.nextDouble()) - c)*UNIT_SIZE);
      _boidsBackup[i] = new Boid(new Vector2(0.0,0.0));
    }
    
    _canvas.onClick.listen((e) {
       _canvas.requestFullscreen();
       _canvas.width = window.innerWidth;
       _canvas.height = window.innerHeight;
     });

    _canvas.onMouseMove.listen((MouseEvent e) {
      Vector2 x = new Vector2(e.client.x / _canvas.width, e.client.y / _canvas.height);
      x -= new Vector2(0.5,0.5);
      x *= UNIT_SIZE;
      x += SCREEN_CENTER;
      
      _target = x;
     });
    _canvas.onMouseLeave.listen((MouseEvent e) {
      _target = null;
    });
  }

  void _advance() {
    Random rng = new Random();
    Vector2 center = new Vector2(0.0, 0.0);
    /* Get the center. */
    for (Boid boid in _boids) {
      center += boid.position;
    }
    center /= _boids.length - 1.0;

    Vector2 momentum = new Vector2(0.0, 0.0);
    for (Boid boid in _boids) {
      momentum += boid.momentum;
    }
    momentum /= _boids.length - 1.0;

    Vector2 rule1(Boid b) {
      Vector2 c = center - b.position * (1.0 / (_boids.length - 1.0));
      return (c-b.position) * FLYING_VELOCITY;
    }

    /* Don't allow boids to fly too close to each other. */
    Vector2 rule2(Boid b) {
      Vector2 c = new Vector2(0.0, 0.0);
      for (Boid boid in _boids) {
        if (boid != b) {
          if ((b.position - boid.position).length < MINIMUM_DISTANCE) {
            c -= boid.position - b.position;
          }
        }
      }
      c += new Vector2(rng.nextDouble() - 0.5, rng.nextDouble() - 0.5) * (2.0 * MINIMUM_DISTANCE);
      return c*AVOIDANCE_VELOCITY;
    }

    /* Boids try to match their velocity vectors with other boids. */
    Vector2 rule3(Boid b) {
      Vector2 c = momentum - b.momentum * (1.0 / (_boids.length - 1.0));
      return (c-b.momentum) * FLYING_ACCELERATION;
    }

    Vector2 attraction;
    
    if(_target != null ) {
      attraction = _target;
    } else {
      attraction = new Vector2(0.0,0.0);
    }
    for (int i = 0; i < _boids.length; i++) {
      Vector2 v4 = attraction - _boids[i].position;
      v4 *= ATTRACTION_VELOCITY * v4.length / UNIT_SIZE;

      Vector2 v1 = rule1(_boids[i]);
      Vector2 v2 = rule2(_boids[i]);
      Vector2 v3 = rule3(_boids[i]);
      _boidsBackup[i].momentum = _boids[i].momentum + v1 + v2 + v3 + v4;
      _boidsBackup[i].position = _boids[i].position + _boids[i].momentum;
      if(_boidsBackup[i].momentum.length > MAXIMUM_VELOCITY) {
        _boidsBackup[i].momentum *=
           MAXIMUM_VELOCITY / _boidsBackup[i].momentum.length;
      }
    }

 
    List<Boid> t = _boids;
    _boids = _boidsBackup;
    _boidsBackup = t;
  }

  void _render() {
    _context.save();
    _context.scale(_canvas.width, _canvas.height);
    _context.clearRect(0, 0, 1, 1);

    _context.translate(0.5 ,0.5);
    _context.scale(1.0 / UNIT_SIZE, 1.0 / UNIT_SIZE);
    _context.translate(-SCREEN_CENTER.x,-SCREEN_CENTER.y);

    double mod(double x, double y) {
      double z = x / y;
      return y * (z - z.floor());
    }
    _context.lineWidth = 10;
    for (Boid boid in _boids) {
      _context.beginPath();
      /*{
      double x = mod(boid.position.x,UNIT_SIZE);  
      double y = mod(boid.position.y,UNIT_SIZE);  
      _context.arc(x,y, BOID_RADIUS, 0, 2 * PI);
      }*/
      _context.arc(boid.position.x, boid.position.y, BOID_RADIUS, 0, 2 * PI);
      _context.stroke();
    }

    _context.restore();
  }
  void loop(Timer timer) {
    _render();
    _advance();
  }
  Timer startTimer() {
    const duration = const Duration(milliseconds: 1000 ~/ TICS_PER_SECOND);

    return new Timer.periodic(duration, loop);
  }
}

void main() {
  Renderer renderer = new Renderer(querySelector("#screen"));
  renderer.startTimer();
}
