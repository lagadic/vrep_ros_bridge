#include <pluginlib/class_list_macros.h>

#include <drawing_handler/DrawMeshHandler.h>
#include <v_repLib.h>
#include <vrep_ros_plugin/access.h>

#include <vrep_ros_plugin/ConsoleHandler.h>

#include <boost/functional/hash.hpp>

const unsigned int DrawMeshHandler::DATA_MAIN = boost::hash<std::string>()("DrawMeshHandler");
const unsigned int DrawMeshHandler::DATA_WIDTH = DrawMeshHandler::DATA_MAIN+1;
const unsigned int DrawMeshHandler::DATA_DIFFUSE = DrawMeshHandler::DATA_WIDTH+1;
const unsigned int DrawMeshHandler::DATA_SPECULAR = DrawMeshHandler::DATA_DIFFUSE+1;
const unsigned int DrawMeshHandler::DATA_EMISSION = DrawMeshHandler::DATA_SPECULAR+1;
const unsigned int DrawMeshHandler::DATA_TRANSPARENCY = DrawMeshHandler::DATA_EMISSION+1;

DrawMeshHandler::DrawMeshHandler() :
		GenericObjectHandler(), _lastTime(0.0), _frequency(-1), _drawingObject(
				-1), _width(10) {

	for (unsigned int i = 0; i < 3; ++i) {
		_diffuse[i] = _specular[i] = _emission[i] = 0;
	}
	_diffuse[0] = 1.0;

	registerCustomVariables();

}

DrawMeshHandler::~DrawMeshHandler() {
}

unsigned int DrawMeshHandler::getObjectType() const {
	return DATA_MAIN;
}

void DrawMeshHandler::synchronize() {
	// We update DrawMeshHandler's data from its associated scene object custom data:
	// 1. Get all the developer data attached to the associated scene object (this is custom data added by the developer):
	int buffSize = simGetObjectCustomDataLength(_associatedObjectID,
			CustomDataHeaders::DEVELOPER_DATA_HEADER);
	char* datBuff = new char[buffSize];
	simGetObjectCustomData(_associatedObjectID,
			CustomDataHeaders::DEVELOPER_DATA_HEADER, datBuff);
	std::vector<unsigned char> developerCustomData(datBuff, datBuff + buffSize);
	delete[] datBuff;
	// 2. From that retrieved data, try to extract sub-data with the DATA_MAIN tag:
	std::vector<unsigned char> tempMainData;

	if (CAccess::extractSerializationData(developerCustomData, DATA_MAIN,
			tempMainData)) { // Yes, the tag is there. For now we only have to synchronize _maxVelocity:

		// Remove # chars for compatibility with ROS
		_associatedObjectName = simGetObjectName(_associatedObjectID);
		std::stringstream streamtemp;
		streamtemp << "- [Draw Mesh] in '" << _associatedObjectName << "'."
				<< std::endl;

	}

}

bool DrawMeshHandler::endOfSimulation() {
	if (_drawingObject > 0) {
		simRemoveDrawingObject(_drawingObject);
		_drawingObject = -1;
	}
	return GenericObjectHandler::endOfSimulation();
}

void DrawMeshHandler::handleSimulation() {
	// called when the main script calls: simHandleModule
	if (!_initialized) {
		_initialize();
	}

	std::stringstream ss;

	const simFloat currentSimulationTime = simGetSimulationTime();
	shape_msgs::MeshConstPtr msg = _lastMsg;
	_lastMsg.reset(); //only draw new meshes TODO: we might loose a message if it arrives between this and the previous line!

	if ((currentSimulationTime - _lastTime) >= 1.0 / _frequency && msg) {

		if (_drawingObject > 0)
			simRemoveDrawingObject(_drawingObject);

		_drawingObject = simAddDrawingObject(
				sim_drawing_triangles + sim_drawing_painttag
						+ sim_drawing_followparentvisibility + _transparency,
				_width, 0.0, _associatedObjectID, msg->triangles.size(),
				_diffuse, NULL, _specular, _emission);

		simFloat data[9];
		for (shape_msgs::Mesh::_triangles_type::const_iterator it =
				msg->triangles.begin(); it != msg->triangles.end(); ++it) {
			for (unsigned int i = 0; i < 3; ++i) {
				data[3 * i + 0] = msg->vertices[it->vertex_indices[i]].x;
				data[3 * i + 1] = msg->vertices[it->vertex_indices[i]].y;
				data[3 * i + 2] = msg->vertices[it->vertex_indices[i]].z;
			}
			simAddDrawingObjectItem(_drawingObject, data);
		}

		_lastTime = currentSimulationTime;

	}

	if (ss.rdbuf()->in_avail()) {
		ConsoleHandler::printInConsole(ss);
	}
}

void DrawMeshHandler::_initialize() {
	if (_initialized)
		return;

	std::vector<unsigned char> developerCustomData;
	getDeveloperCustomData(developerCustomData);
	std::vector<unsigned char> tempMainData;

	std::stringstream ss;

	if (CAccess::extractSerializationData(developerCustomData, DATA_MAIN,
			tempMainData)) {
		_frequency = CAccess::pop_float(tempMainData);
		ss << "- [" << _associatedObjectName << "] Drawing frequency set to "
				<< _frequency << "." << std::endl;
	} else {
		_frequency = -1;
		ss << "- [" << _associatedObjectName
				<< "] Drawing frequency not specified. Using simulation step as default."
				<< std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData, DATA_WIDTH,
			tempMainData)) {
		_width = CAccess::pop_int(tempMainData);
		ss << "- [" << _associatedObjectName << "] Line width set to " << _width
				<< "." << std::endl;
	} else {
		_width = 1;
		ss << "- [" << _associatedObjectName
				<< "] Line width not specified. Using " << _width << " as default"
				<< std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData, DATA_DIFFUSE,
			tempMainData)) {
		const std::vector<simFloat> tmp = CAccess::pop_float(tempMainData, 3);
		if (tmp.size() == 3) {
			memcpy(_diffuse, tmp.data(), 3 * sizeof(simFloat));
			ss << "- [" << _associatedObjectName << "] Diffuse color set to ["
					<< _diffuse[0] << ", " << _diffuse[1] << ", " << _diffuse[2]
					<< "]." << std::endl;
		}
	} else {
		ss << "- [" << _associatedObjectName
				<< "] Diffuse color not specified. Using [" << _diffuse[0]
				<< ", " << _diffuse[1] << ", " << _diffuse[2] << "] as default"
				<< std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData, DATA_SPECULAR,
			tempMainData)) {
		const std::vector<simFloat> tmp = CAccess::pop_float(tempMainData, 3);
		if (tmp.size() == 3) {
			memcpy(_specular, tmp.data(), 3 * sizeof(simFloat));
			ss << "- [" << _associatedObjectName << "] Specular color set to ["
					<< _specular[0] << ", " << _specular[1] << ", "
					<< _specular[2] << "]." << std::endl;
		}
	} else {
		ss << "- [" << _associatedObjectName
				<< "] Specular color not specified. Using [" << _specular[0]
				<< ", " << _specular[1] << ", " << _specular[2]
				<< "] as default" << std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData, DATA_EMISSION,
			tempMainData)) {
		const std::vector<simFloat> tmp = CAccess::pop_float(tempMainData, 3);
		if (tmp.size() == 3) {
			memcpy(_emission, tmp.data(), 3 * sizeof(simFloat));
			ss << "- [" << _associatedObjectName << "] Emission color set to ["
					<< _emission[0] << ", " << _emission[1] << ", "
					<< _emission[2] << "]." << std::endl;
		}
	} else {
		ss << "- [" << _associatedObjectName
				<< "] Emission color not specified. Using [" << _emission[0]
				<< ", " << _emission[1] << ", " << _emission[2]
				<< "] as default" << std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData,
			DATA_TRANSPARENCY, tempMainData)) {
		_transparency = CAccess::pop_int(tempMainData);
		switch (_transparency) {
		case 0:
			_transparency = 0;
			ss << "- [" << _associatedObjectName
					<< "] Transparency set to 100%." << std::endl;
			break;
		case 1:
			_transparency = sim_drawing_50percenttransparency;
			ss << "- [" << _associatedObjectName << "] Transparency set to 50%."
					<< std::endl;
			break;
		case 2:
			_transparency = sim_drawing_25percenttransparency;
			ss << "- [" << _associatedObjectName << "] Transparency set to 25%."
					<< std::endl;
			break;
		case 3:
			_transparency = sim_drawing_12percenttransparency;
			ss << "- [" << _associatedObjectName
					<< "] Transparency set to 12.5%." << std::endl;
			break;
		default:
			ss << "- [" << _associatedObjectName
					<< "] The specified transparency value (" << _transparency
					<< ") is not valid. Transparency value can only be 0 (100%), 1 (50%), 2 (25%), or 3 (12.5). Using 100% as default."
					<< std::endl;
			_transparency = 0;
		}
	} else {
		_transparency = 0;
		ss << "- [" << _associatedObjectName
				<< "] Transparency not specified. Using 100% as default"
				<< std::endl;
	}

	std::string topicName(_associatedObjectName);
	std::replace(topicName.begin(), topicName.end(), '#', '_');
	topicName += "/DrawMesh";
	_sub = _nh.subscribe(topicName, 1, &DrawMeshHandler::msgCallback, this);

	ss << "- [" << _associatedObjectName
			<< "] Waiting for mesh triangles on topic " << topicName << "."
			<< std::endl;
	;
	ConsoleHandler::printInConsole(ss);

	_lastTime = -1e5;
	_initialized = true;
}

void DrawMeshHandler::msgCallback(shape_msgs::MeshConstPtr msg) {
	_lastMsg = msg;
}

void DrawMeshHandler::registerCustomVariables() const {
#if VREP_VERSION_MAJOR*10000 + VREP_VERSION_MINOR*100 + VREP_VERSION_PATCH < 3*10000 + 3*100 + 1
	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_mesh_data_main",
			(boost::lexical_cast<std::string>(int(DATA_MAIN))).c_str());
	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_mesh_data_width",
			(boost::lexical_cast<std::string>(int(DATA_WIDTH))).c_str());
	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_mesh_data_diffuse",
			(boost::lexical_cast<std::string>(int(DATA_DIFFUSE))).c_str());
	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_mesh_data_specular",
			(boost::lexical_cast<std::string>(int(DATA_SPECULAR))).c_str());
	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_mesh_data_emission",
			(boost::lexical_cast<std::string>(int(DATA_EMISSION))).c_str());
	simRegisterCustomLuaVariable(
			"sim_ext_ros_bridge_draw_mesh_data_transparency",
			(boost::lexical_cast<std::string>(int(DATA_TRANSPARENCY))).c_str());
#else
	simRegisterScriptVariable("sim_ext_ros_bridge_draw_mesh_data_main",
			(boost::lexical_cast<std::string>(int(DATA_MAIN))).c_str(),0);
	simRegisterScriptVariable("sim_ext_ros_bridge_draw_mesh_data_width",
			(boost::lexical_cast<std::string>(int(DATA_WIDTH))).c_str(),0);
	simRegisterScriptVariable("sim_ext_ros_bridge_draw_mesh_data_diffuse",
			(boost::lexical_cast<std::string>(int(DATA_DIFFUSE))).c_str(),0);
	simRegisterScriptVariable("sim_ext_ros_bridge_draw_mesh_data_specular",
			(boost::lexical_cast<std::string>(int(DATA_SPECULAR))).c_str(),0);
	simRegisterScriptVariable("sim_ext_ros_bridge_draw_mesh_data_emission",
			(boost::lexical_cast<std::string>(int(DATA_EMISSION))).c_str(),0);
	simRegisterScriptVariable("sim_ext_ros_bridge_draw_mesh_data_transparency",
			(boost::lexical_cast<std::string>(int(DATA_TRANSPARENCY))).c_str(),0);
#endif	
}

PLUGINLIB_EXPORT_CLASS(DrawMeshHandler, GenericObjectHandler)
