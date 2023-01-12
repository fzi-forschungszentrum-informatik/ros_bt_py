#  -------- BEGIN LICENSE BLOCK --------
#  Copyright 2022 FZI Forschungszentrum Informatik
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#     * Neither the name of the {copyright_holder} nor the names of its
#        contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#  -------- END LICENSE BLOCK --------
"""Module implementing the ParallelAuctionManager.

This auction manager runs multiple single item auctions in parallel and assigns robots to
task based on the lowest bid.
"""
import dataclasses
import threading
import uuid
from typing import Dict

import rospy
from rospy import ServiceException
from ros_bt_py_msgs.msg import AuctionMessage
from ros_bt_py_msgs.srv import (
    FindBestCapabilityExecutorRequest,
    FindBestCapabilityExecutorResponse,
    GetLocalBid,
    GetLocalBidResponse,
    GetLocalBidRequest,
    GetAvailableRemoteCapabilitySlots,
    ReserveRemoteCapabilitySlot,
    GetAvailableRemoteCapabilitySlotsRequest,
    GetAvailableRemoteCapabilitySlotsResponse,
    ReserveRemoteCapabilitySlotRequest,
)
from ros_bt_py.assignment_manager.assignment_manager import AssignmentManager


@dataclasses.dataclass
class Bid:
    """Contains all information about a bid submitted by a robot."""

    bid: float
    """The bid value submitted by the robot."""

    executors: int
    """The number of available executors on the robot."""

    implementation: str
    """The implmentation on the robot the bid was genertated for."""


class AuctionStatus:
    """Status of a running auction.

    This class contains all information used by the AssignmentManager to track auctions.
    These auctions can be run by the robot themselves or a remote robot.
    """

    def __init__(self, auction_id: str, auctioneer_id: str, deadline: rospy.Time):
        """Create a new record of an auction for a specific auction.

        :param auction_id: The unique id of the auction within the current robot team.
        :type auction_id: str
        :param auctioneer_id: The name of the robot running the auction as the auctioneer.
        :type auctioneer_id: str
        :param deadline: The timestamp until bids can be submitted to the auction.
        :type deadline: rospy.Time
        """
        self.__auction_id = auction_id
        self.__auctioneer_id = auctioneer_id
        self.__deadline = deadline
        self.__is_closed = False
        self.__bids: Dict[str, Bid] = {}
        self.__auction_status_lock = threading.Lock()

    @property
    def auction_id(self):
        """Unique id of the auction within the current robot team.

        :return: The id string of the auction.
        :rtype: str
        """
        return self.__auction_id

    @property
    def auctioneer_id(self):
        """Unique robot name of the auctioneer for the auction.

        :return: The name of the robot.
        :rtype: str
        """
        return self.__auctioneer_id

    @property
    def deadline(self):
        """The deadline of the auction until all bids have to be submitted.

        :return: The timestamp of the deadline.
        :rtype: rospy.Time
        """
        return self.__deadline

    @deadline.setter
    def deadline(self, new_deadline: rospy.Time):
        """Set the deadline of the auction until all bids have to be submitted.

        :param new_deadline: The new deadline timestamp for the auction.
        :type new_deadline: rospy.Time
        """
        with self.__auction_status_lock:
            self.__deadline = new_deadline

    @property
    def is_closed(self):
        """If the auction was closed by the auctioneer.

        :return: If the auction is closed.
        :rtype: bool
        """
        with self.__auction_status_lock:
            return self.__is_closed

    @is_closed.setter
    def is_closed(self, is_closed: bool):
        """Set if the auction is closed and no more bids can be accepted.

        :param is_closed: The new status of the auction.
        :type is_closed: bool
        """
        with self.__auction_status_lock:
            self.__is_closed = is_closed

    def get_valid_bids(self):
        """Get all valid bids submitted until now.

        A valid bid is a bid where the robot reported at least one available executor.
        This function is thread safe.

        :return: A list of tuples containing the id of the remote robot and is submitted bid.
        :rtype: List[Tuple[str, float]]
        """
        with self.__auction_status_lock:
            return list(
                map(
                    lambda y: (y, self.__bids[y].bid),
                    filter(lambda x: self.__bids[x].executors > 0, self.__bids),
                )
            )

    def get_bid(self, robot_name: str):
        """Get a specific bid submitted by a robot for this auciton.

        :param robot_name: The name of the robot to retrieve the bid from.
        :type robot_name: str
        :raises exc: KeyError if the robot has not submitted a bid until now.
        :return: The bid value submitted by the robot.
        :rtype: float
        """
        with self.__auction_status_lock:
            try:
                return self.__bids[robot_name]
            except KeyError as exc:
                rospy.logwarn(f"Failed to receive the bid for robot {robot_name}")
                raise exc

    def set_bid(
        self, robot_name: str, bid: float, no_executors: int, implementation_name: str
    ):
        """Set the bid value for a specifc robot.

        :param robot_name: Name of the robot submitting a bid.
        :type robot_name: str
        :param bid: The bid value for the robots best implmentation.
        :type bid: float
        :param no_executors: The number of executors available for the robot
        to run the implementation.
        :type no_executors: int
        :param implementation_name: The name of the implmentation the robot will use.
        :type implementation_name: str
        """
        with self.__auction_status_lock:
            self.__bids[robot_name] = Bid(
                bid=bid, executors=no_executors, implementation=implementation_name
            )
            rospy.logdebug(f"Set bid: {robot_name} {self.__bids[robot_name]}")


class ParallelAuctionManager(AssignmentManager):
    """Parallel assignment manager class.

    This auction manager runs multiple single item auctions in parallel and assigns robots to
    task based on the lowest bid.
    """

    running_auctions: Dict[str, AuctionStatus] = {}

    def __init__(
        self,
        local_topic_prefix: str,
        global_assignment_msg_topic_prefix: str,
    ):
        """Create a new ParallelAuctionManager using the given topic prefixes for its pub/sub's.

        :param local_topic_prefix: The prefix for communicating with the local robots.
        :type local_topic_prefix: str
        :param global_assignment_msg_topic_prefix: The prefix for communicating with other robots
        AssignmentManagers.
        :type global_assignment_msg_topic_prefix: str
        """
        super().__init__(
            local_topic_prefix=local_topic_prefix,
            global_assignment_msg_topic_prefix=global_assignment_msg_topic_prefix,
        )

        self.__local_topic_prefix = local_topic_prefix
        self.__auction_duration = rospy.Duration.from_sec(
            rospy.get_param("auction_duration_sec", 15.0)
        )

        self.__global_auction_message_pub = rospy.Publisher(
            global_assignment_msg_topic_prefix,
            AuctionMessage,
            latch=False,
            queue_size=1,
        )
        self.__global_auction_message_sub = rospy.Subscriber(
            global_assignment_msg_topic_prefix,
            AuctionMessage,
            self.global_auction_messages_callback,
            queue_size=1,
        )

        local_bid_service_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/get_local_bid",
        )

        self.__local_bid_service = rospy.ServiceProxy(
            local_bid_service_topic, GetLocalBid
        )
        self.__local_bid_service_lock = threading.RLock()

        available_remote_slots_service_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/get_available_remote_slots",
        )

        self.__available_remote_capability_slots_service = rospy.ServiceProxy(
            available_remote_slots_service_topic, GetAvailableRemoteCapabilitySlots
        )

        reserve_remote_slots_service_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/reserve_remote_capability_slot",
        )

        self.__reserve_remote_capability_slot_service = rospy.ServiceProxy(
            reserve_remote_slots_service_topic, ReserveRemoteCapabilitySlot
        )
        self.running_auctions_lock = threading.RLock()

    def __get_available_local_remote_capability_slots(self) -> int:
        try:
            response: GetAvailableRemoteCapabilitySlotsResponse = (
                self.__available_remote_capability_slots_service.call(
                    GetAvailableRemoteCapabilitySlotsRequest()
                )
            )
            return response.available_remote_capability_slots
        except ServiceException as exc:
            rospy.logerr(f"Failed to receive the available remote slots: {exc}")
            return 0

    def _is_running_auction(self, auction_id: str):
        return not self.running_auctions[auction_id].is_closed

    def __handle_announcement_msg(self, msg: AuctionMessage):
        with self.running_auctions_lock:
            self.running_auctions[msg.auction_id] = AuctionStatus(
                auction_id=msg.auction_id,
                auctioneer_id=msg.sender_id,
                deadline=msg.deadline,
            )
        with self.__local_bid_service_lock:
            try:
                bid_response: GetLocalBidResponse = self.__local_bid_service.call(
                    GetLocalBidRequest(
                        interface=msg.interface,
                        inputs_topic=msg.inputs_topic,
                        outputs_topic=msg.outputs_topic,
                    )
                )
                if not bid_response.success:
                    rospy.logwarn(
                        f"Failed to get local bid from: {self.__local_topic_prefix}, "
                        f"{bid_response.error_message}",
                        logger_name="assignment_system",
                    )
                    return
            except ServiceException as exc:
                rospy.logerr(f"Cannot calculate local bid, service error: {exc}")
                return

        available_executors: int = self.__get_available_local_remote_capability_slots()

        with self.running_auctions_lock:
            self.running_auctions[msg.auction_id].set_bid(
                robot_name=self.__local_topic_prefix,
                bid=bid_response.bid,
                no_executors=available_executors,
                implementation_name=bid_response.implementation_name,
            )

        self.update_executor_numbers_for_running_auctions(
            available_executors=available_executors
        )

        return

    def __handle_bid_msg(self, msg: AuctionMessage):
        try:
            with self.running_auctions_lock:
                auction_status = self.running_auctions[msg.auction_id]
        except KeyError:
            rospy.logerr(
                "Unknown auction detected! Ignoring as not all required information is present!"
            )
            return

        if auction_status.auctioneer_id != self.__local_topic_prefix:
            rospy.logdebug("Received bid for foreign auction, ignoring!")
            return

        if auction_status.is_closed or msg.timestamp > auction_status.deadline:
            rospy.logwarn(
                f"Ignoring bid from {msg.sender_id} for auction {msg.auction_id}, "
                "as it arrived after the deadline!"
            )
            return

        rospy.logfatal(
            f"Received bid: {msg.auction_id} {msg.sender_id} {msg.bid} {msg.number_of_executors}"
        )

        auction_status.set_bid(
            robot_name=msg.sender_id,
            bid=msg.bid,
            implementation_name=msg.implementation_name,
            no_executors=msg.number_of_executors,
        )
        with self.running_auctions_lock:
            self.running_auctions[msg.auction_id] = auction_status

    def __handle_close_msg(self, msg: AuctionMessage):
        try:
            with self.running_auctions_lock:
                self.running_auctions[msg.auction_id].is_closed = True
        except KeyError:
            rospy.logerr(f"Auction {msg.auction_id} not found, ignoring!")

    def __handle_abort_msg(self, msg: AuctionMessage):
        try:
            with self.running_auctions_lock:
                del self.running_auctions[msg.auction_id]
        except KeyError:
            rospy.logerr(f"Auction {msg.auction_id} not found, ignoring!")

    def __handle_result_msg(self, msg: AuctionMessage):
        if msg.result_id == self.__local_topic_prefix:
            rospy.loginfo("Got awarded an auction!")

            available_slots = self.__get_available_local_remote_capability_slots()
            if available_slots >= 1:
                with self.running_auctions_lock:
                    for auction_id in filter(
                        self._is_running_auction, self.running_auctions
                    ):
                        status = self.running_auctions[auction_id]
                        try:
                            local_bid = status.get_bid(self.__local_topic_prefix)
                        except KeyError:
                            continue

                        bid_msg = AuctionMessage(
                            auction_id=auction_id,
                            timestamp=rospy.Time.now(),
                            sender_id=self.__local_topic_prefix,
                            message_type=AuctionMessage.BID,
                            bid=local_bid.bid,
                            number_of_executors=available_slots - 1,
                            implementation_name=local_bid.implementation,
                        )
                        self.__global_auction_message_pub.publish(bid_msg)

            local_bid = self.running_auctions[msg.auction_id].get_bid(
                self.__local_topic_prefix
            )

            rospy.logfatal(f"Top bid: {local_bid} Second Highest Bid: {msg.bid}")
            self.__reserve_remote_capability_slot_service.call(
                ReserveRemoteCapabilitySlotRequest(
                    remote_mission_control=msg.sender_id,
                    implementation_name=local_bid.implementation,
                    reauction_threshold=msg.bid,
                )
            )

            del self.running_auctions[msg.auction_id]
            return

        with self.running_auctions_lock:
            try:
                local_bid = self.running_auctions[msg.auction_id].get_bid(
                    self.__local_topic_prefix
                )
            except KeyError:
                return
            if local_bid.executors > 0:
                rospy.logfatal(
                    f"Reallocating executor slots after losing {msg.auction_id} auction!"
                )

                available_executors = (
                    self.__get_available_local_remote_capability_slots()
                )

                self.update_executor_numbers_for_running_auctions(
                    available_executors=available_executors
                )

            del self.running_auctions[msg.auction_id]
            return

    def __handle_update_msg(self, msg: AuctionMessage):
        pass

    def update_executor_numbers_for_running_auctions(self, available_executors: int):
        """
        Update all running auctions with the new number of available executors.

        :param available_executors: The new number of available executors.
        :return: None
        """
        with self.running_auctions_lock:
            unsorted_running_auction_by_bid = []
            for auction_id in self.running_auctions:
                if self._is_running_auction(auction_id):
                    try:
                        self.running_auctions[auction_id].get_bid(
                            self.__local_topic_prefix
                        )
                    except KeyError:
                        continue
                    unsorted_running_auction_by_bid.append(auction_id)

            running_auctions_by_bid = list(
                sorted(
                    unsorted_running_auction_by_bid,
                    key=lambda x: self.running_auctions[x]
                    .get_bid(self.__local_topic_prefix)
                    .bid,
                )
            )

            possible_auctions = running_auctions_by_bid[:available_executors]
            impossible_auctions = running_auctions_by_bid[available_executors:]

            for auction_id in impossible_auctions:
                auction = self.running_auctions[auction_id]
                local_bid = auction.get_bid(self.__local_topic_prefix)

                rospy.logfatal(
                    f"Publishing bid: {auction.auction_id} {local_bid.bid} 0"
                )
                bid_msg = AuctionMessage(
                    auction_id=auction.auction_id,
                    timestamp=rospy.Time.now(),
                    sender_id=self.__local_topic_prefix,
                    message_type=AuctionMessage.BID,
                    bid=local_bid.bid,
                    number_of_executors=0,
                    implementation_name=local_bid.implementation,
                )
                self.__global_auction_message_pub.publish(bid_msg)

            for auction_id in possible_auctions:
                auction = self.running_auctions[auction_id]
                local_bid = auction.get_bid(self.__local_topic_prefix)

                rospy.logfatal(
                    f"Publishing bid: {auction.auction_id} {local_bid.bid} {available_executors}"
                )
                bid_msg = AuctionMessage(
                    auction_id=auction.auction_id,
                    timestamp=rospy.Time.now(),
                    sender_id=self.__local_topic_prefix,
                    message_type=AuctionMessage.BID,
                    bid=local_bid.bid,
                    number_of_executors=available_executors,
                    implementation_name=local_bid.implementation,
                )
                self.__global_auction_message_pub.publish(bid_msg)

    def global_auction_messages_callback(self, msg: AuctionMessage):
        """Handle messages on the global auction messages topic as a callback.

        This callback handles auction information coming from other robots, handling them
        appropriately.

        :param msg: The message received on the topic that should be handled.
        :type msg: AuctionMessage
        """
        if msg.sender_id == self.__local_topic_prefix:
            rospy.logdebug_throttle(1, f"Ignoring local message: {msg.sender_id}")
            return

        if msg.message_type == msg.ANNOUNCEMENT:
            rospy.logdebug(f"Handling announcement auction message {msg.auction_id}")
            self.__handle_announcement_msg(msg)
            return
        if msg.message_type == msg.BID:
            rospy.logdebug(f"Handling bid auction message {msg.auction_id}")
            self.__handle_bid_msg(msg)
            return
        if msg.message_type == msg.CLOSE:
            rospy.logdebug(f"Handling close auction message {msg.auction_id}")
            self.__handle_close_msg(msg)
            return
        if msg.message_type == msg.ABORT:
            rospy.logdebug(f"Handling abort auction message {msg.auction_id}")
            self.__handle_abort_msg(msg)
            return
        if msg.message_type == msg.RESULT:
            rospy.logdebug(f"Handling result auction message {msg.auction_id}")
            self.__handle_result_msg(msg)
            return
        if msg.message_type == msg.UPDATE:
            rospy.logdebug(f"Handling update auction message {msg.auction_id}")
            self.__handle_update_msg(msg)
            return

    def find_best_capability_executor(
        self, goal: FindBestCapabilityExecutorRequest
    ) -> FindBestCapabilityExecutorResponse:
        """Find the best executor and implementation for a specifc capability interface.

        To this end an auction is stated and the best available team member is found to execute
        the interfaces action.

        :param goal: The goal containing the capability interface that should be executed.
        :type goal: FindBestCapabilityExecutorRequest
        :return: Response containing the optimal assignment.
        :rtype: FindBestCapabilityExecutorResponse
        """
        response = FindBestCapabilityExecutorResponse()

        current_auction_id = str(uuid.uuid4())
        current_deadline = rospy.Time.now() + self.__auction_duration

        with self.running_auctions_lock:
            self.running_auctions[current_auction_id] = AuctionStatus(
                auction_id=current_auction_id,
                auctioneer_id=self.__local_topic_prefix,
                deadline=current_deadline,
            )

        self.__global_auction_message_pub.publish(
            AuctionMessage(
                auction_id=current_auction_id,
                timestamp=rospy.Time.now(),
                sender_id=self.__local_topic_prefix,
                message_type=AuctionMessage.ANNOUNCEMENT,
                interface=goal.capability,
                inputs_topic=goal.inputs_topic,
                outputs_topic=goal.outputs_topic,
                deadline=current_deadline,
            )
        )

        rospy.sleep(self.__auction_duration + rospy.Duration.from_sec(0.02))

        self.__global_auction_message_pub.publish(
            AuctionMessage(
                auction_id=current_auction_id,
                timestamp=rospy.Time.now(),
                sender_id=self.__local_topic_prefix,
                message_type=AuctionMessage.CLOSE,
            )
        )
        with self.running_auctions_lock:
            bids = self.running_auctions[current_auction_id].get_valid_bids()

        if len(bids) < 1:
            response.success = False
            response.error_message = "No valid bids received! Auction has failed!"
            rospy.logerr(response.error_message)
            return response

        sorted_bids = sorted(bids, key=lambda x: x[1])
        top_bid = sorted_bids[0]
        top_bid_id = top_bid[0]

        if len(sorted_bids) > 1:
            second_highest_bid = sorted_bids[0][1] * 1.2
        else:
            second_highest_bid = top_bid[1] * 1.2

        rospy.logfatal(
            f"Auction result {current_auction_id} {top_bid_id} {top_bid[1]}!"
        )

        top_bid_info = self.running_auctions[current_auction_id].get_bid(top_bid_id)

        self.__global_auction_message_pub.publish(
            AuctionMessage(
                auction_id=current_auction_id,
                timestamp=rospy.Time.now(),
                sender_id=self.__local_topic_prefix,
                message_type=AuctionMessage.RESULT,
                bid=second_highest_bid,
                result_id=top_bid_id,
                implementation_name=top_bid_info.implementation,
            )
        )

        rospy.sleep(rospy.Duration.from_sec(1))

        response.success = True
        response.implementation_name = top_bid_info.implementation
        response.execute_local = top_bid_id == self.__local_topic_prefix
        response.executor_mission_control_topic = top_bid_id

        return response
