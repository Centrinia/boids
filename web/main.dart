// Copyright (c) 2015, <your name>. All rights reserved. Use of this source code
// is governed by a BSD-style license that can be found in the LICENSE file.

import 'dart:math';
import 'dart:html';
import 'dart:async';
import 'package:vector_math/vector_math.dart';

class Body {
  Vector2 _position;
  Vector2 _momentum;
  double _mass;

  get mass => _mass;
  get position => _position;
  get velocity => _momentum / _mass;
  get momentum => _momentum;

  set position(Vector2 x) => _position = new Vector2.copy(x);
  set momentum(Vector2 p) => _momentum = new Vector2.copy(p);
  set velocity(Vector2 v) => _momentum = v * _mass;

  Body(Vector2 position, Vector2 momentum, double mass) {
    _position = new Vector2.copy(position);
    _momentum = new Vector2.copy(momentum);
    _mass = mass;
  }

  Body.copy(Body body) {
    _position = new Vector2.copy(body.position);
    _momentum = new Vector2.copy(body.momentum);
    _mass = body.mass;
  }

  void advance() {
    _position += _momentum / _mass;
  }
}
class Boid {
  static const RADIUS = 20.0;
  static const DENSITY = 1.0 / (40.0 * PI);
  Body _body;

  get body => _body;
  set body(Body body) => _body = body;

  Boid(Vector2 x, Vector2 p) {
    _body = new Body(x, p, RADIUS * RADIUS * PI * DENSITY);
  }
  Boid.copy(Boid boid) {
    _body = new Body.copy(boid.body);
  }
}

class Flock {
  static const UNIT_SIZE = 10000.0;
  static const FLYING_VELOCITY = 1.0 / 400.0;
  static const ATTRACTION_VELOCITY = 1.0 / 50.0 / UNIT_SIZE;
  static const AVOIDANCE_VELOCITY = 1.0;
  static const FLYING_ACCELERATION = 1.0 / 40.0;
  static const MINIMUM_DISTANCE = 30.0;
  static const MAXIMUM_VELOCITY = 100.0;

  Vector2 _target = null;
  set target(Vector2 target) => _target = target;

  List<Boid> _boids;

  get boids => _boids;

  Flock(int count) {
    _boids = new List<Boid>(count);

    Random rng = new Random();
    Vector2 c = new Vector2(0.5, 0.5);
    for (int i = 0; i < _boids.length; i++) {
      _boids[i] = new Boid(
          (new Vector2(rng.nextDouble(), rng.nextDouble()) - c) * UNIT_SIZE,
          new Vector2.zero());
    }
  }

  void advance() {
    Random rng = new Random();
    Vector2 center = new Vector2(0.0, 0.0);
    /* Get the center. */
    for (Boid boid in _boids) {
      center += boid.body.position;
    }
    center /= _boids.length - 1.0;

    Vector2 momentum = new Vector2(0.0, 0.0);
    for (Boid boid in _boids) {
      momentum += boid.body.momentum;
    }
    momentum /= _boids.length - 1.0;

    Vector2 rule1(Boid b) {
      Vector2 c = center - b.body.position * (1.0 / (_boids.length - 1.0));
      return (c - b.body.position) * FLYING_VELOCITY;
    }

    /* Don't allow boids to fly too close to each other. */
    Vector2 rule2(Boid b) {
      Vector2 c = new Vector2(0.0, 0.0);
      for (Boid boid in _boids) {
        if (boid != b) {
          if ((b.body.position - boid.body.position).length <
              MINIMUM_DISTANCE) {
            c -= boid.body.position - b.body.position;
          }
        }
      }
      c += new Vector2(rng.nextDouble() - 0.5, rng.nextDouble() - 0.5) *
          (2.0 * MINIMUM_DISTANCE);
      return c * AVOIDANCE_VELOCITY;
    }

    /* Boids try to match their velocity vectors with other boids. */
    Vector2 rule3(Boid b) {
      Vector2 c = momentum - b.body.momentum * (1.0 / (_boids.length - 1.0));
      return (c - b.body.momentum) * FLYING_ACCELERATION;
    }

    Vector2 attraction;

    if (_target != null) {
      attraction = _target;
    } else {
      attraction = new Vector2(0.0, 0.0);
    }
    for (int i = 0; i < _boids.length; i++) {
      Vector2 v4 = attraction - _boids[i].body.position;
      v4 *= ATTRACTION_VELOCITY * v4.length;

      Vector2 v1 = rule1(_boids[i]);
      Vector2 v2 = rule2(_boids[i]);
      Vector2 v3 = rule3(_boids[i]);

      _boids[i].body.velocity = _boids[i].body.velocity + v1 + v2 + v3 + v4;
      if (_boids[i].body.velocity.length > MAXIMUM_VELOCITY) {
        _boids[i].body.velocity *=
            MAXIMUM_VELOCITY / _boids[i].body.velocity.length;
      }
    }

    for (Boid b in _boids) {
      b.body.advance();
    }
  }
}

class Player {
  static const DAMPENING = 0.7;
  static const DENSITY = 1.0;
  static const RADIUS = 50.0;
  static const FLAIL_DENSITY = 1e-5;
  static const FLAIL_RADIUS = 200.0;
  static const MAXIMUM_VELOCITY = 200.0;
  static const MAXIMUM_FLAIL_VELOCITY = 700.0;
  static const STRING_LENGTH = 2000.0;
  Body _body;
  Body _flail;

  get body => _body;
  get flail => _flail;
  Vector2 _target;

  Player(Vector2 position) {
    _body =
        new Body(position, new Vector2.zero(), RADIUS * RADIUS * PI * DENSITY);
    _flail = new Body(position, new Vector2.zero(),
        FLAIL_RADIUS * FLAIL_RADIUS * PI * FLAIL_DENSITY);
    _target = null;
  }
  void moveToward(Vector2 c) {
    _target = c;
  }

  void advance() {
    _body.momentum *= DAMPENING;
    _flail.momentum *= DAMPENING;

    if (_target != null) {
      _body.velocity += _target - body.position;
      if (_body.velocity.length > MAXIMUM_VELOCITY) {
        _body.velocity *= MAXIMUM_VELOCITY / _body.velocity.length;
      }
    }

    Vector2 displacement = _flail.position - _body.position;
    if (displacement.length > STRING_LENGTH) {
      /*Vector2 p = _flail.momentum + _body.momentum;
      
      _flail.momentum = p / 2.0;
      _body.momentum = p/2.0;*/
      //_flail.momentum += _body.momentum/2;

      //_flail.position = _body.position + displacement * (STRING_LENGTH / displacement.length);
      _flail.velocity += -displacement * (STRING_LENGTH / displacement.length);
      if (_flail.velocity.length > MAXIMUM_FLAIL_VELOCITY) {
        _flail.velocity *= MAXIMUM_FLAIL_VELOCITY / _flail.velocity.length;
      }
    }

    _flail.advance();
    _body.advance();
  }
}
class Renderer {
  static final SCREEN_CENTER = new Vector2(10.0, 10.0);
  static const TICS_PER_SECOND = 35;

  CanvasElement _canvas;
  CanvasRenderingContext2D _context;

  Flock _flock;
  Player _player;
  Vector2 _dragTarget;

  Vector2 canvasToWorld(Point p) {
    Vector2 x = new Vector2(p.x / _canvas.width, p.y / _canvas.height);
    x -= new Vector2(0.5, 0.5);
    x *= Flock.UNIT_SIZE;
    x += SCREEN_CENTER;
    return x;
  }
  Renderer(CanvasElement canvas) {
    const BOID_COUNT = 50;

    _player = new Player(new Vector2(0.0, 0.0));
    _flock = new Flock(BOID_COUNT);
    _canvas = canvas;
    _context = _canvas.context2D;

    _dragTarget = null;
    _canvas.onMouseDown.listen((MouseEvent e) {
      _dragTarget = canvasToWorld(e.client);
      _flock.target = _player.body.position;
    });
    _canvas.onMouseMove.listen((MouseEvent e) {
      if (_dragTarget != null) {
        _dragTarget = canvasToWorld(e.client);
      }
    });
    _canvas.onMouseUp.listen((MouseEvent e) {
      _dragTarget = null;
    });

    _canvas.onMouseLeave.listen((MouseEvent e) {
      _dragTarget = null;
    });
  }

  void _render() {
    _context.save();
    _context.scale(_canvas.width, _canvas.height);
    _context.clearRect(0, 0, 1, 1);

    _context.translate(0.5, 0.5);
    _context.scale(1.0 / Flock.UNIT_SIZE, 1.0 / Flock.UNIT_SIZE);
    _context.translate(-SCREEN_CENTER.x, -SCREEN_CENTER.y);

    _context.lineWidth = 10;
    for (Boid boid in _flock.boids) {
      _context.beginPath();
      _context.arc(
          boid.body.position.x, boid.body.position.y, Boid.RADIUS, 0, 2 * PI);
      _context.stroke();
    }

    /* Draw the player. */
    _context.beginPath();
    _context.arc(_player.body.position.x, _player.body.position.y,
        Player.RADIUS, 0, 2 * PI);
    _context.stroke();

    /* Draw the flail. */
    _context.beginPath();
    _context.arc(_player.flail.position.x, _player.flail.position.y,
        Player.FLAIL_RADIUS, 0, 2 * PI);
    _context.stroke();
    /* Draw the string. */
    _context.beginPath();
    _context.moveTo(_player.body.position.x, _player.body.position.y);

    _context.lineTo(_player.flail.position.x, _player.flail.position.y);

    _context.stroke();

    _context.restore();
  }
  void loop(Timer timer) {
    _render();
    if (_dragTarget != null) {
      _player.moveToward(_dragTarget);
      _flock.target = _player.body._position;
    }

    _flock.advance();
    _player.advance();
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
