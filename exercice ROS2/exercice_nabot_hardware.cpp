#include "nabot_base/nabot_hardware.hpp"

namespace
{
    // ON A BESOIN ICI d'UN CONTAINER AVEC UNE INSTANCE DE ROBOCLAW
}

namespace nabot_base
{
    static const std::string HW_NAME = // NOM DE NOTRE SYSTEM POUR ROS2_CONTROL;
    static const std::string LEFT_CMD_JOINT_NAME = // JOINTS DES ROUES DE GAUCHE;
    static const std::string RIGHT_CMD_JOINT_NAME = // JOINTS DES ROUES DE DROITE;

	
	// CONFIGURATION
    hardware_interface::return_type NabotSystemHardware::configure(const hardware_interface::HardwareInfo & info)
    {

        if (configure_default(info) != hardware_interface::return_type::OK)
        {
            return hardware_interface::return_type::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Name: %s", info_.name.c_str());

        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Number of Joints %u", info_.joints.size());

        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_position_offset_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

		// RECUPERER DU MODELE DE ROBOT LES PARAMETRES NECESSAIRES
        // exemple
		// param_ = std::stoi(info_.hardware_parameters["nom_du_param"]);

        // AFFICHER LA VERSION DU MICRO-LOGICIEL DU ROBOCLAW
        roboclaw.displayVersion(address_);

        // METTRE EN MEMOIRE LES DONNEES DES PID (versatil - s'efface a chaque coupure d'alim du roboclaw)
        // VERIFIER QUE LA COMMANDE DE MISE EN MEMOIRE A ABOUTI
		// AFFICHER LES VALUERS MISES EN MEMOIRE

        // FAIRE UN RESET DES ENCODEURS - TOUJOURS VERIFIER QUE LA COMMANDE ABOUTI

        // Update hardware interface structure
        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            // NabotSystemHardware has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(HW_NAME),
                    "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(HW_NAME),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(HW_NAME),
                    "Joint '%s' has %d state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(HW_NAME),
                    "Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.",
                    joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_POSITION);
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(HW_NAME),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::return_type::ERROR;
            }
            }

        status_ = hardware_interface::status::CONFIGURED;
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> NabotSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> NabotSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));

            // Detmerine which joints will be used for commands
            if (info_.joints[i].name == LEFT_CMD_JOINT_NAME)
            {
            left_cmd_joint_index_ = i;
            }

            if (info_.joints[i].name == RIGHT_CMD_JOINT_NAME)
            {
            right_cmd_joint_index_ = i;
            }
        }

        return command_interfaces;
    }

    /**
     * RECUPERE LA VITESSE et LA POSITION DES MOTEURS GRACE AU ROBOCLAW
	 * REPERCUTE LES VALEURS DANS LE MODELE
     */
    void NabotSystemHardware::updateJointsFromHardware()
    {
		// CREER DES CONTAINERS SOUS LA FORME DE VECTEURS (ATTENTION A BIEN TYPER LE CONTENU)
		// POUR CHACUNES DES INFORMATIONS :
		//			nombre de tocks des encodeurs
		//			vitesse des encodeurs
		//			statut (pour capturer les sauts de registre, voir documentation roboclaw)
		//			statut de la vitesse (voir documentation roboclaw)
        std::vector<uint32_t> encoders;
        std::vector<uint32_t> encoders_speed;
        std::vector<uint8_t> status;
        std::vector<uint8_t> speed_status;

		// LIRE LES INFOS AVEC l'INSTANCE DE ROBOCLAW
		// VERIFIER QUE LA LECTURE S'EST BIEN PASSEE

        // TRAITER LA DONNEE
		//// TRANSFORMER LES TICKS EN ANGLE
		////// GERER LES SAUTS DE REGISTRE
		//// TRANSFORMER LES TICKS/S EN RAD/S

        // A COMPLETER SI LE COEUR VOUS EN DIT
        // AVEC LA VERIF d'INFO DE DIAGNOSTIQUE (TENSION BATTERIE, TEMPERATURES MOTEURS...)
    }


  void NabotSystemHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }

  void NabotSystemHardware::writeCommandsToHardware()
  {
	// FONCTION QUI ENVOI LES ORDRES AUX MOTEURS
    double diff_speed_left = // recuperer la consigne du moteur gauche;
    double diff_speed_right = // recuperer la consigne du moteur droit;
	// ON LIMITE PAR LE LOGICIEL LE DIFFERENTIEL DE VITESSE
    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    // ENVOYER LA CONSIGNE AUX MOTEURS GRACE A VOTRE INSTANCE de ROBOCLAW
  }

    hardware_interface::return_type NabotSystemHardware::start()
    {
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Starting ...please wait...");

        // ON NE FAIT QU'INITIALISER A ZERO TOUTES LES GRANDEURS
        for (auto i = 0u; i < hw_states_position_.size(); i++)
        {
            if (std::isnan(hw_states_position_[i]))
            {
            hw_states_position_[i] = 0;
            hw_states_position_offset_[i] = 0;
            hw_states_velocity_[i] = 0;
            hw_commands_[i] = 0;
            }
        }

        status_ = hardware_interface::status::STARTED;
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System Successfully started!");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type NabotSystemHardware::stop()
    {
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Stopping ...please wait...");
        status_ = hardware_interface::status::STOPPED;
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System successfully stopped!");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type NabotSystemHardware::read()
    {
		// RECUPERATION DES INFOS DES ENCODEURS
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Reading from hardware");
        // ON APPELLE ICI LA FONCTION QUI VA FAIRE LE JOB 
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully read!");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type NabotSystemHardware::write()
    {
		// ENVOI DES CONSIGNES MOTEURS AU HARDWARE
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Writing to hardware");
        // ON APPELLE ICI LA FONCTION QUI VA FAIRE LE JOB 
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully written!");
        return hardware_interface::return_type::OK;
    }


}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    nabot_base::NabotSystemHardware, hardware_interface::SystemInterface)
