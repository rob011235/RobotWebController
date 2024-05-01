
# A very simple Flask Hello World app for you to get started with...

from flask import Flask, redirect, render_template, request, url_for
from flask_sqlalchemy import SQLAlchemy

app = Flask(__name__)

app.config["DEBUG"] = True

SQLALCHEMY_DATABASE_URI = "mysql+mysqlconnector://{username}:{password}@{hostname}/{databasename}".format(
    username="USERNAME",
    password="PASSWORD",
    hostname="rob011235.mysql.pythonanywhere-services.com",
    databasename="rob011235$RobotCommandDB",
)
app.config["SQLALCHEMY_DATABASE_URI"] = SQLALCHEMY_DATABASE_URI
app.config["SQLALCHEMY_POOL_RECYCLE"] = 299
app.config["SQLALCHEMY_TRACK_MODIFICATIONS"] = False
db = SQLAlchemy(app)

class Command(db.Model):

    __tablename__ = "commands"

    id = db.Column(db.Integer, primary_key=True)
    text = db.Column(db.String(4096))
    timestamp = db.Column(db.DateTime)

@app.route("/", methods = ["GET","POST"])
def index():
    if request.method == "GET":
        return render_template("main_page.html",commands = Command.query.all())

    command = Command(text=request.form["text"], timestamp = request.form["timestamp"])
    db.session.add(command)
    db.session.commit()
    return redirect(url_for('index'))
